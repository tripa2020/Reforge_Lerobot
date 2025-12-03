#!/usr/bin/env python3
"""
LeRobot Phase 2 CSV Parser with Integrated Analysis

Captures binary sensor data from Teensy 4.1 via Serial4 and automatically
runs timing/coherency analysis when capture completes.

Features:
  - Same handshake protocol as Phase 1 (READY → START → ACK)
  - 64-byte Phase 2 struct format (Seqlock diagnostics)
  - Real-time sample counting with progress display
  - Automatic analysis on Ctrl+C or --duration timeout
  - Timing plots saved to PNG

Usage:
  # Basic capture (Ctrl+C to stop, then auto-analyze)
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv -v

  # Timed capture (auto-stops after 60 seconds, then analyzes)
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv --duration 60

  # Skip analysis
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv --no-analyze

Author: Alex + Claude
Date: 2025-12-02
Phase: 2.0 (Seqlock + Servo @ 500Hz)
"""

import struct
import serial
import sys
import argparse
import signal
import time
import os

# ============== PHASE 2 DATA STRUCTURE (64 bytes) ==============
# Must match SensorData struct in Lerobot_RM_Phase2.ino

SAMPLE_SIZE = 64

# Struct format (little-endian):
# I: timestamp_us (4 bytes)
# I: generation (4 bytes)
# 6f: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z (24 bytes)
# 2i: servo_position, servo_velocity (8 bytes)
# 4H: imu_read_us, servo_read_us, isr_total_us, imu_servo_skew_us (8 bytes)
# h: period_error_us (2 bytes) - SIGNED
# 2x: padding for alignment (2 bytes) - compiler adds this before checksum
# I: checksum (4 bytes)
# B: coherency_flag (1 byte)
# B: servo_error_code (1 byte)
# 6x: reserved padding (6 bytes)
# Total: 4+4+24+8+8+2+2+4+1+1+6 = 64 bytes ✓

STRUCT_FMT = "<I I 6f 2i 4H h 2x I B B 6x"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

# Verify struct size matches expected
assert STRUCT_SIZE == SAMPLE_SIZE, f"Struct size mismatch: {STRUCT_SIZE} != {SAMPLE_SIZE}"

# Field names for CSV header
CSV_HEADER = (
    "timestamp_sec,generation,"
    "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
    "servo_pos,servo_vel,"
    "imu_read_us,servo_read_us,isr_total_us,imu_servo_skew_us,"
    "period_error_us,checksum,coherency_flag,servo_error_code"
)


def parse_sample(data):
    """Parse 64-byte binary sample into dict."""
    fields = struct.unpack(STRUCT_FMT, data)
    return {
        'timestamp_sec': fields[0] / 1e6,
        'generation': fields[1],
        'accel_x': fields[2],
        'accel_y': fields[3],
        'accel_z': fields[4],
        'gyro_x': fields[5],
        'gyro_y': fields[6],
        'gyro_z': fields[7],
        'servo_pos': fields[8],
        'servo_vel': fields[9],
        'imu_read_us': fields[10],
        'servo_read_us': fields[11],
        'isr_total_us': fields[12],
        'imu_servo_skew_us': fields[13],
        'period_error_us': fields[14],  # Signed!
        'checksum': fields[15],
        'coherency_flag': fields[16],
        'servo_error_code': fields[17],
    }


def sample_to_csv_row(sample):
    """Convert sample dict to CSV row string."""
    return (
        f"{sample['timestamp_sec']:.6f},{sample['generation']},"
        f"{sample['accel_x']:.6f},{sample['accel_y']:.6f},{sample['accel_z']:.6f},"
        f"{sample['gyro_x']:.6f},{sample['gyro_y']:.6f},{sample['gyro_z']:.6f},"
        f"{sample['servo_pos']},{sample['servo_vel']},"
        f"{sample['imu_read_us']},{sample['servo_read_us']},"
        f"{sample['isr_total_us']},{sample['imu_servo_skew_us']},"
        f"{sample['period_error_us']},{sample['checksum']},"
        f"{sample['coherency_flag']},{sample['servo_error_code']}"
    )


# ============== INTEGRATED ANALYSIS ==============

def analyze_phase2(csv_file, save_plots=True):
    """
    Analyze Phase 2 CSV data and print validation results.
    Optionally saves timing plots to PNG.
    """
    try:
        import pandas as pd
        import numpy as np
    except ImportError:
        print("[WARN] pandas/numpy not installed - skipping analysis", file=sys.stderr)
        print("[WARN] Install with: pip install pandas numpy matplotlib", file=sys.stderr)
        return None

    print("\n" + "=" * 60)
    print("LeRobot Phase 2.0 Validation Analysis")
    print("=" * 60)

    # Load CSV
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"[ERROR] Failed to load CSV: {e}", file=sys.stderr)
        return None

    print(f"\n[INFO] Loaded {len(df)} samples from {csv_file}")

    if len(df) < 10:
        print("[ERROR] Not enough samples for analysis (need >= 10)")
        return None

    # ========== BASIC STATS ==========
    print("\n" + "-" * 60)
    print("1. BASIC STATISTICS")
    print("-" * 60)

    duration = df['timestamp_sec'].max() - df['timestamp_sec'].min()
    actual_rate = len(df) / duration if duration > 0 else 0

    print(f"Duration: {duration:.2f} seconds")
    print(f"Actual rate: {actual_rate:.1f} Hz (target: 500 Hz)")
    print(f"Total samples: {len(df)}")

    # ========== DATA INTEGRITY ==========
    print("\n" + "-" * 60)
    print("2. DATA INTEGRITY (Dropped Samples)")
    print("-" * 60)

    generation_diffs = df['generation'].diff()
    drops = (generation_diffs > 1).sum()
    total_dropped = int((generation_diffs[generation_diffs > 1] - 1).sum()) if drops > 0 else 0

    print(f"Dropped samples: {total_dropped}")
    print(f"Drop events: {drops}")
    drop_rate = (total_dropped / (len(df) + total_dropped)) * 100 if len(df) > 0 else 0
    print(f"Drop rate: {drop_rate:.4f}%")

    drop_pass = total_dropped == 0
    print(f"Zero drops: {'✅ PASS' if drop_pass else '❌ FAIL'}")

    # ========== JITTER ANALYSIS ==========
    print("\n" + "-" * 60)
    print("3. JITTER ANALYSIS (Period Error)")
    print("-" * 60)

    print(f"Mean: {df['period_error_us'].mean():.2f} µs")
    print(f"Std: {df['period_error_us'].std():.2f} µs")
    print(f"Max: {df['period_error_us'].max():.0f} µs")
    print(f"Min: {df['period_error_us'].min():.0f} µs")

    max_jitter = max(abs(df['period_error_us'].max()), abs(df['period_error_us'].min()))
    jitter_pass = max_jitter < 100
    print(f"Jitter < 100µs: {'✅ PASS' if jitter_pass else '❌ FAIL'}")

    # ========== WCET ANALYSIS ==========
    print("\n" + "-" * 60)
    print("4. WCET ANALYSIS (ISR Execution Time)")
    print("-" * 60)

    print(f"Max: {df['isr_total_us'].max()} µs")
    print(f"Mean: {df['isr_total_us'].mean():.1f} µs")
    print(f"Std: {df['isr_total_us'].std():.1f} µs")
    print(f"99.9th percentile: {df['isr_total_us'].quantile(0.999):.1f} µs")

    wcet_pass = df['isr_total_us'].max() < 400
    print(f"WCET < 400µs: {'✅ PASS' if wcet_pass else '❌ FAIL'}")

    # ========== SENSOR SKEW ANALYSIS ==========
    print("\n" + "-" * 60)
    print("5. SENSOR SKEW (IMU-Servo Time Delta)")
    print("-" * 60)

    print(f"Mean: {df['imu_servo_skew_us'].mean():.1f} µs")
    print(f"Std: {df['imu_servo_skew_us'].std():.1f} µs")
    print(f"Min: {df['imu_servo_skew_us'].min()} µs")
    print(f"Max: {df['imu_servo_skew_us'].max()} µs")

    skew_deterministic = df['imu_servo_skew_us'].std() < 10
    print(f"Skew variance < 10µs: {'✅ PASS' if skew_deterministic else '❌ FAIL'}")

    # ========== COHERENCY (TORN READS) ==========
    print("\n" + "-" * 60)
    print("6. COHERENCY (Seqlock Torn Reads)")
    print("-" * 60)

    torn = (df['coherency_flag'] == 0xFF).sum()
    stale = (df['coherency_flag'] == 0xEE).sum()
    valid = (df['coherency_flag'] == 0xAA).sum()

    print(f"Valid samples (0xAA): {valid}")
    print(f"Torn reads (0xFF): {torn}")
    print(f"Stale reads (0xEE): {stale}")

    coherency_pass = torn == 0
    print(f"Zero torn reads: {'✅ PASS' if coherency_pass else '❌ FAIL'}")

    # ========== SERVO ERRORS ==========
    print("\n" + "-" * 60)
    print("7. SERVO COMMUNICATION")
    print("-" * 60)

    servo_errors = (df['servo_error_code'] != 0).sum()
    servo_timeouts = (df['servo_error_code'] == 1).sum()
    servo_checksum = (df['servo_error_code'] == 2).sum()
    servo_framing = (df['servo_error_code'] == 3).sum()

    print(f"Total errors: {servo_errors} / {len(df)} ({servo_errors/len(df)*100:.2f}%)")
    print(f"  Timeouts (1): {servo_timeouts}")
    print(f"  Checksum (2): {servo_checksum}")
    print(f"  Framing (3): {servo_framing}")

    servo_pass = (servo_errors / len(df)) < 0.03  # < 3% error rate
    print(f"Servo errors < 3%: {'✅ PASS' if servo_pass else '❌ FAIL'}")

    # ========== COMPONENT READ TIMES ==========
    print("\n" + "-" * 60)
    print("8. COMPONENT READ TIMES")
    print("-" * 60)

    print(f"IMU read:")
    print(f"  Mean: {df['imu_read_us'].mean():.1f} µs")
    print(f"  Max: {df['imu_read_us'].max()} µs")

    print(f"Servo read:")
    print(f"  Mean: {df['servo_read_us'].mean():.1f} µs")
    print(f"  Max: {df['servo_read_us'].max()} µs")

    # ========== OVERALL RESULT ==========
    print("\n" + "=" * 60)
    print("PHASE 2.0 VALIDATION SUMMARY")
    print("=" * 60)

    all_pass = drop_pass and jitter_pass and wcet_pass and skew_deterministic and coherency_pass and servo_pass

    results = {
        'Zero drops': drop_pass,
        'Jitter < 100µs': jitter_pass,
        'WCET < 400µs': wcet_pass,
        'Skew std < 10µs': skew_deterministic,
        'Zero torn reads': coherency_pass,
        'Servo errors < 3%': servo_pass,
    }

    for name, passed in results.items():
        status = '✅ PASS' if passed else '❌ FAIL'
        print(f"  {name}: {status}")

    print("\n" + "=" * 60)
    if all_pass:
        print("✅ ALL TESTS PASSED - Phase 2.0 Validated!")
    else:
        print("❌ SOME TESTS FAILED - Review issues above")
    print("=" * 60)

    # ========== GENERATE PLOTS ==========
    if save_plots:
        try:
            import matplotlib.pyplot as plt

            plot_file = csv_file.replace('.csv', '_analysis.png')

            fig, axes = plt.subplots(4, 1, figsize=(14, 12))

            # Jitter
            axes[0].plot(df['timestamp_sec'], df['period_error_us'], linewidth=0.5)
            axes[0].axhline(0, color='r', linestyle='--')
            axes[0].axhline(100, color='orange', linestyle=':')
            axes[0].axhline(-100, color='orange', linestyle=':')
            axes[0].set_ylabel('Period Error (µs)')
            axes[0].set_title('Jitter @ 500Hz (target: ±100µs)')
            axes[0].grid(True, alpha=0.3)

            # WCET
            axes[1].plot(df['timestamp_sec'], df['isr_total_us'], linewidth=0.5)
            axes[1].axhline(df['isr_total_us'].mean(), color='g', linestyle='--', label='Mean')
            axes[1].axhline(400, color='r', linestyle='--', label='Budget (400µs)')
            axes[1].set_ylabel('ISR Time (µs)')
            axes[1].set_title('ISR Execution Time (WCET)')
            axes[1].legend()
            axes[1].grid(True, alpha=0.3)

            # Skew
            axes[2].plot(df['timestamp_sec'], df['imu_servo_skew_us'], linewidth=0.5)
            axes[2].axhline(df['imu_servo_skew_us'].mean(), color='g', linestyle='--', label='Mean')
            axes[2].set_ylabel('Skew (µs)')
            axes[2].set_title('IMU-Servo Time Skew (should be constant)')
            axes[2].legend()
            axes[2].grid(True, alpha=0.3)

            # Component breakdown
            axes[3].plot(df['timestamp_sec'], df['imu_read_us'], label='IMU', alpha=0.7, linewidth=0.5)
            axes[3].plot(df['timestamp_sec'], df['servo_read_us'], label='Servo', alpha=0.7, linewidth=0.5)
            axes[3].set_xlabel('Time (s)')
            axes[3].set_ylabel('Read Time (µs)')
            axes[3].set_title('Sensor Read Times')
            axes[3].legend()
            axes[3].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(plot_file, dpi=150)
            print(f"\n[INFO] Plot saved: {plot_file}")

        except ImportError:
            print("[WARN] matplotlib not installed - skipping plots", file=sys.stderr)
        except Exception as e:
            print(f"[WARN] Failed to generate plots: {e}", file=sys.stderr)

    return results


# ============== MAIN CAPTURE FUNCTION ==============

def main():
    parser = argparse.ArgumentParser(
        description="LeRobot Phase 2 CSV Parser with Integrated Analysis",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic capture (Ctrl+C to stop, then auto-analyze)
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv -v

  # Timed capture (auto-stops after 60 seconds)
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv --duration 60

  # Skip analysis
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv --no-analyze
        """
    )
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyACM0)")
    parser.add_argument("baud", type=int, help="Baud rate (e.g., 2000000)")
    parser.add_argument("-o", "--output", required=True, help="Output CSV file")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--duration", type=int, default=0, help="Capture duration in seconds (0=indefinite)")
    parser.add_argument("--no-analyze", action="store_true", help="Skip automatic analysis after capture")
    parser.add_argument("--no-plots", action="store_true", help="Skip generating plots")
    args = parser.parse_args()

    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        if args.verbose:
            print(f"[INFO] Opened {args.port} @ {args.baud} baud", file=sys.stderr)
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open serial port: {e}", file=sys.stderr)
        sys.exit(1)

    # Open output file
    try:
        outfile = open(args.output, 'w')
        outfile.write(CSV_HEADER + "\n")
    except IOError as e:
        print(f"[ERROR] Failed to open output file: {e}", file=sys.stderr)
        ser.close()
        sys.exit(1)

    # ============== HANDSHAKE PROTOCOL ==============
    if args.verbose:
        print(f"[INFO] Waiting for READY from Teensy...", file=sys.stderr)

    handshake_complete = False
    handshake_timeout = 30  # 30 second timeout
    start_time = time.time()

    while not handshake_complete:
        # Check timeout
        if time.time() - start_time > handshake_timeout:
            print(f"[ERROR] Handshake timeout - no READY received", file=sys.stderr)
            print(f"[ERROR] Check Teensy USB connection and upload", file=sys.stderr)
            ser.close()
            outfile.close()
            sys.exit(1)

        # Read line from Serial4
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('ascii').strip()

                if line == "READY":
                    if args.verbose:
                        print(f"[INFO] Received READY from Teensy", file=sys.stderr)
                        print(f"[INFO] Sending START command...", file=sys.stderr)

                    # Send START command
                    ser.write(b"START\n")
                    ser.flush()

                    # Wait for ACK
                    ack_timeout = 5
                    ack_start = time.time()

                    while time.time() - ack_start < ack_timeout:
                        if ser.in_waiting > 0:
                            ack_line = ser.readline().decode('ascii').strip()
                            if ack_line == "ACK":
                                handshake_complete = True
                                if args.verbose:
                                    print(f"[INFO] Handshake complete - logging started!", file=sys.stderr)
                                break

                    if not handshake_complete:
                        print(f"[ERROR] No ACK received from Teensy", file=sys.stderr)
                        ser.close()
                        outfile.close()
                        sys.exit(1)

            except UnicodeDecodeError:
                pass  # Ignore binary data during handshake

    # ============== CAPTURE LOOP ==============
    buffer = b""
    sample_count = 0
    parse_errors = 0
    capture_start = time.time()
    last_status_time = time.time()
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        running = False
        if args.verbose:
            print(f"\n[INFO] Capture stopped by user", file=sys.stderr)

    signal.signal(signal.SIGINT, signal_handler)

    if args.verbose:
        if args.duration > 0:
            print(f"[INFO] Capturing for {args.duration} seconds...", file=sys.stderr)
        else:
            print(f"[INFO] Capturing (Ctrl+C to stop)...", file=sys.stderr)

    while running:
        # Check duration timeout
        if args.duration > 0 and (time.time() - capture_start) > args.duration:
            if args.verbose:
                print(f"\n[INFO] Duration reached ({args.duration}s)", file=sys.stderr)
            break

        # Read available data
        waiting = ser.in_waiting
        if waiting > 0:
            buffer += ser.read(waiting)
        else:
            buffer += ser.read(1)  # Block briefly

        # Process complete samples
        while len(buffer) >= SAMPLE_SIZE:
            chunk = buffer[:SAMPLE_SIZE]
            buffer = buffer[SAMPLE_SIZE:]

            try:
                sample = parse_sample(chunk)
                row = sample_to_csv_row(sample)
                outfile.write(row + "\n")
                sample_count += 1

            except struct.error as e:
                parse_errors += 1
                if args.verbose and parse_errors <= 10:
                    print(f"[WARN] Parse error #{parse_errors}: {e}", file=sys.stderr)

        # Status update every 2 seconds
        if args.verbose and (time.time() - last_status_time) > 2.0:
            elapsed = time.time() - capture_start
            rate = sample_count / elapsed if elapsed > 0 else 0
            print(f"[INFO] {sample_count} samples, {rate:.1f} Hz, {elapsed:.1f}s elapsed", file=sys.stderr)
            last_status_time = time.time()

    # ============== CLEANUP ==============
    ser.close()
    outfile.close()

    elapsed = time.time() - capture_start
    final_rate = sample_count / elapsed if elapsed > 0 else 0

    print(f"\n[INFO] Capture complete!", file=sys.stderr)
    print(f"[INFO] Samples: {sample_count}", file=sys.stderr)
    print(f"[INFO] Duration: {elapsed:.2f} seconds", file=sys.stderr)
    print(f"[INFO] Rate: {final_rate:.1f} Hz", file=sys.stderr)
    print(f"[INFO] Parse errors: {parse_errors}", file=sys.stderr)
    print(f"[INFO] Output: {args.output}", file=sys.stderr)

    # ============== AUTO-ANALYZE ==============
    if not args.no_analyze and sample_count > 0:
        analyze_phase2(args.output, save_plots=not args.no_plots)


if __name__ == "__main__":
    main()
