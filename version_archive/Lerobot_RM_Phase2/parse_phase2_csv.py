#!/usr/bin/env python3
"""
LeRobot Phase 2.1.2 CSV Parser with Integrated Analysis

Captures binary sensor data from Teensy 4.1 via Serial4 and automatically
runs timing/coherency analysis when capture completes.

Phase 2.1.2 Architecture:
  - Background servo polling @ ~526 Hz (non-blocking FSM in main loop)
  - IMU sampling @ 500 Hz (bare-metal ISM330_Bare.h driver in ISR)
  - Seqlock synchronization for lock-free ISR→main communication
  - Servo age tracking for offline interpolation

Features:
  - Same handshake protocol as Phase 1 (READY → START → ACK)
  - 64-byte Phase 2.1.2 struct format (Seqlock + servo age diagnostics)
  - Real-time sample counting with progress display
  - Automatic analysis on Ctrl+C or --duration timeout
  - Timing plots saved to PNG
  - Servo age validation (background polling architecture)
  - Bare-metal IMU driver performance validation

Usage:
  # Basic capture (Ctrl+C to stop, then auto-analyze)
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv -v

  # Timed capture (auto-stops after 60 seconds, then analyzes)
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv --duration 60

  # Skip analysis
  python3 parse_phase2_csv.py /dev/ttyACM0 2000000 -o test.csv --no-analyze

Author: Alex + Claude
Date: 2025-12-05
Phase: 2.1.2 (Background servo polling + bare-metal IMU)
"""

import struct
import serial
import sys
import argparse
import signal
import time
import os

# ============== PHASE 2.1.2 DATA STRUCTURE (64 bytes) ==============
# Must match SensorData struct in Lerobot_RM_Phase2.ino
# Phase 2.1.2: Background servo polling + IMU @ 500Hz

SAMPLE_SIZE = 64

# Struct format (little-endian):
# I: timestamp_us (4 bytes) - ISR entry time
# I: generation (4 bytes) - Monotonic counter (detect drops)
# 6f: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z (24 bytes) - m/s² and rad/s
# 2i: servo_position, servo_velocity (8 bytes) - position 0-4095, velocity reserved
# 4H: imu_read_us, servo_age_us, isr_total_us, imu_servo_skew_us (8 bytes)
#     NOTE: servo_read_us is ACTUALLY servo_age_us (age of servo sample in µs)
#     NOTE: imu_servo_skew_us is UNUSED in Phase 2.1.2 (always 0)
# h: period_error_us (2 bytes) - SIGNED jitter (actual period - 2000µs)
# 2x: padding for alignment (2 bytes) - compiler adds this before checksum
# I: checksum (4 bytes) - XOR checksum for torn read detection
# B: coherency_flag (1 byte) - 0xAA=valid, 0xFF=torn, 0xEE=stale
# B: servo_error_code (1 byte) - 0=OK, 1=timeout, 2=checksum, 3=framing
# 6x: reserved padding (6 bytes)
# Total: 4+4+24+8+8+2+2+4+1+1+6 = 64 bytes ✓

STRUCT_FMT = "<I I 6f 2i 4H h 2x I B B 6x"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

# Verify struct size matches expected
assert STRUCT_SIZE == SAMPLE_SIZE, f"Struct size mismatch: {STRUCT_SIZE} != {SAMPLE_SIZE}"

# Field names for CSV header
# Phase 2.1.2: servo_read_us is ACTUALLY servo_age_us (age of servo sample)
CSV_HEADER = (
    "timestamp_sec,generation,"
    "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
    "servo_pos,servo_vel,"
    "imu_read_us,servo_age_us,isr_total_us,imu_servo_skew_us,"
    "period_error_us,checksum,coherency_flag,servo_error_code"
)


def parse_sample(data):
    """
    Parse 64-byte binary sample into dict.
    Phase 2.1.2: Background servo polling + IMU @ 500Hz ISR
    """
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
        'servo_age_us': fields[11],  # Age of servo sample (not read time!)
        'isr_total_us': fields[12],
        'imu_servo_skew_us': fields[13],  # Unused in Phase 2.1.2 (always 0)
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
        f"{sample['imu_read_us']},{sample['servo_age_us']},"
        f"{sample['isr_total_us']},{sample['imu_servo_skew_us']},"
        f"{sample['period_error_us']},{sample['checksum']},"
        f"{sample['coherency_flag']},{sample['servo_error_code']}"
    )


# ============== INTEGRATED ANALYSIS ==============

def analyze_phase2(csv_file, save_plots=True):
    """
    Analyze Phase 2.1.2 CSV data and print validation results.
    Phase 2.1.2: Background servo polling + IMU @ 500Hz ISR
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
    print("LeRobot Phase 2.1.2 Validation Analysis")
    print("=" * 60)
    print("Architecture: Background servo polling (~526 Hz) + IMU @ 500 Hz ISR")
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

    # ========== STARTUP TRANSIENT FILTERING ==========
    # Drop row 0: startup transient (timestamp often 0, coherency may be invalid)
    if len(df) > 1:
        df = df.iloc[1:].reset_index(drop=True)
        print(f"[INFO] Dropped row 0 (startup transient), analyzing {len(df)} samples")

    # ========== BASIC STATS ==========
    print("\n" + "-" * 60)
    print("1. BASIC STATISTICS")
    print("-" * 60)

    # Use device timestamps (not wall-clock capture time)
    duration = df['timestamp_sec'].max() - df['timestamp_sec'].min()
    actual_rate = len(df) / duration if duration > 0 else 0

    print(f"Duration: {duration:.2f} seconds (from device timestamps)")
    print(f"Actual rate: {actual_rate:.1f} Hz (target: 500 Hz)")
    print(f"Total samples: {len(df)}")

    rate_pass = 490 <= actual_rate <= 510  # ±2% tolerance
    print(f"Rate within 490-510 Hz: {'✅ PASS' if rate_pass else '❌ FAIL'}")

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

    # ========== SERVO AGE ANALYSIS (Phase 2.1.2) ==========
    print("\n" + "-" * 60)
    print("5. SERVO AGE (Background Polling Architecture)")
    print("-" * 60)
    print("NOTE: Servo polled in background (~526 Hz), ISR snapshots latest value")
    print("      servo_age_us = age of servo sample at each 500 Hz IMU tick")
    print("      Analysis ONLY includes samples with error_code==0 (valid servo data)")

    # Filter to VALID servo samples only (no errors, no invalid markers)
    valid_servo = df[(df['servo_error_code'] == 0) & (df['servo_age_us'] != 0xFFFF)]
    total_errors = (df['servo_error_code'] != 0).sum()

    if len(valid_servo) > 0:
        print(f"\nValid servo samples: {len(valid_servo)} / {len(df)} ({len(valid_servo)/len(df)*100:.1f}%)")
        print(f"Error samples (excluded): {total_errors} ({total_errors/len(df)*100:.2f}%)")

        print(f"\nServo age (valid samples only):")
        print(f"  Mean: {valid_servo['servo_age_us'].mean():.1f} µs")
        print(f"  Std: {valid_servo['servo_age_us'].std():.1f} µs")
        print(f"  Min: {valid_servo['servo_age_us'].min()} µs")
        print(f"  Max: {valid_servo['servo_age_us'].max()} µs")

        # For background polling, we expect age to vary 0-1900µs (poll interval)
        max_age = valid_servo['servo_age_us'].max()
        age_reasonable = max_age < 2500  # Allow some margin over 1900µs poll interval
        print(f"\nMax age < 2500µs: {'✅ PASS' if age_reasonable else '❌ FAIL (polling too slow)'}")

        if total_errors > 0:
            # Show what age looks like during error cases (for context, not validation)
            error_servo = df[(df['servo_error_code'] != 0) & (df['servo_age_us'] != 0xFFFF)]
            if len(error_servo) > 0:
                print(f"\n[INFO] Servo age during errors (not counted in pass/fail):")
                print(f"  Mean: {error_servo['servo_age_us'].mean():.1f} µs")
                print(f"  Max: {error_servo['servo_age_us'].max()} µs")
                print(f"  (Expected: ~1 extra poll interval after timeout)")
    else:
        print("[WARN] No valid servo samples (all errors or 0xFFFF)")
        age_reasonable = False

    # ========== COHERENCY (TORN READS) ==========
    print("\n" + "-" * 60)
    print("6. COHERENCY (Seqlock Torn Reads)")
    print("-" * 60)
    print("NOTE: Row 0 already dropped (startup transient)")
    print("      Analyzing only steady-state samples")

    torn = (df['coherency_flag'] == 0xFF).sum()
    stale = (df['coherency_flag'] == 0xEE).sum()
    valid = (df['coherency_flag'] == 0xAA).sum()

    print(f"\nValid samples (0xAA): {valid}")
    print(f"Torn reads (0xFF): {torn}")
    print(f"Stale reads (0xEE): {stale}")

    coherency_pass = torn == 0
    print(f"\nZero torn reads: {'✅ PASS' if coherency_pass else '❌ FAIL'}")

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
    print("8. IMU READ TIME (Bare-Metal Driver)")
    print("-" * 60)

    print(f"IMU read time (SPI burst, 14 bytes):")
    print(f"  Mean: {df['imu_read_us'].mean():.1f} µs")
    print(f"  Max: {df['imu_read_us'].max()} µs")
    print(f"  Min: {df['imu_read_us'].min()} µs")
    print(f"  Std: {df['imu_read_us'].std():.1f} µs")

    imu_fast = df['imu_read_us'].mean() < 100
    print(f"Mean IMU read < 100µs: {'✅ PASS' if imu_fast else '❌ FAIL (bare-metal driver issue)'}")

    # ========== OVERALL RESULT ==========
    print("\n" + "=" * 60)
    print("PHASE 2.1.2 VALIDATION SUMMARY")
    print("=" * 60)

    all_pass = (rate_pass and drop_pass and jitter_pass and wcet_pass and
                age_reasonable and coherency_pass and servo_pass and imu_fast)

    results = {
        'Sample rate 490-510 Hz': rate_pass,
        'Zero drops': drop_pass,
        'Jitter < 100µs': jitter_pass,
        'WCET < 400µs': wcet_pass,
        'Servo age < 2500µs (valid only)': age_reasonable,
        'Zero torn reads': coherency_pass,
        'Servo errors < 3%': servo_pass,
        'IMU read < 100µs': imu_fast,
    }

    for name, passed in results.items():
        status = '✅ PASS' if passed else '❌ FAIL'
        print(f"  {name}: {status}")

    print("\n" + "=" * 60)
    if all_pass:
        print("✅ ALL TESTS PASSED - Phase 2.1.2 Validated!")
        print("   Background servo polling + bare-metal IMU working correctly!")
    else:
        print("❌ SOME TESTS FAILED - Review issues above")
    print("=" * 60)

    # ========== GENERATE PLOTS ==========
    if save_plots:
        try:
            import matplotlib.pyplot as plt

            # ===== TIMING/DIAGNOSTIC PLOTS (4 subplots) =====
            plot_file = csv_file.replace('.csv', '_analysis.png')
            fig1, axes1 = plt.subplots(4, 1, figsize=(16, 12))

            # Jitter
            axes1[0].plot(df['timestamp_sec'].values, df['period_error_us'].values, linewidth=0.5)
            axes1[0].axhline(0, color='r', linestyle='--')
            axes1[0].axhline(100, color='orange', linestyle=':')
            axes1[0].axhline(-100, color='orange', linestyle=':')
            axes1[0].set_ylabel('Period Error (µs)')
            axes1[0].set_title('Jitter @ 500Hz (target: ±100µs)')
            axes1[0].grid(True, alpha=0.3)

            # WCET
            axes1[1].plot(df['timestamp_sec'].values, df['isr_total_us'].values, linewidth=0.5)
            axes1[1].axhline(df['isr_total_us'].mean(), color='g', linestyle='--', label='Mean')
            axes1[1].axhline(400, color='r', linestyle='--', label='Budget (400µs)')
            axes1[1].set_ylabel('ISR Time (µs)')
            axes1[1].set_title('ISR Execution Time (WCET)')
            axes1[1].legend()
            axes1[1].grid(True, alpha=0.3)

            # Servo Age (Phase 2.1.2 - background polling)
            # Filter to valid servo samples (avoid multi-dimensional indexing issue)
            valid_servo_plot = df[df['servo_age_us'] != 0xFFFF].copy()
            if len(valid_servo_plot) > 0:
                axes1[2].plot(valid_servo_plot['timestamp_sec'].values,
                            valid_servo_plot['servo_age_us'].values,
                            linewidth=0.5, alpha=0.7)
                axes1[2].axhline(1900, color='orange', linestyle='--', label='Poll interval (1900µs)')
                axes1[2].set_ylabel('Servo Age (µs)')
                axes1[2].set_title('Servo Sample Age (Background Polling @ ~526 Hz)')
                axes1[2].legend()
                axes1[2].grid(True, alpha=0.3)
            else:
                axes1[2].text(0.5, 0.5, 'No valid servo data', ha='center', va='center',
                            transform=axes1[2].transAxes)
                axes1[2].set_title('Servo Sample Age (No Valid Data)')
                axes1[2].grid(True, alpha=0.3)

            # IMU read time (bare-metal driver)
            axes1[3].plot(df['timestamp_sec'].values, df['imu_read_us'].values, label='IMU SPI read',
                        alpha=0.7, linewidth=0.5, color='blue')
            axes1[3].set_xlabel('Time (s)')
            axes1[3].set_ylabel('Read Time (µs)')
            axes1[3].set_title('IMU Read Time (Bare-Metal Driver)')
            axes1[3].axhline(df['imu_read_us'].mean(), color='g', linestyle='--',
                           label=f'Mean ({df["imu_read_us"].mean():.1f} µs)')
            axes1[3].legend()
            axes1[3].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(plot_file, dpi=150)
            print(f"\n[INFO] Timing plots saved: {plot_file}")
            plt.close(fig1)

            # ===== SENSOR DATA PLOTS (2 subplots) =====
            sensor_plot_file = csv_file.replace('.csv', '_sensor_data.png')
            fig2, axes2 = plt.subplots(2, 1, figsize=(16, 8))

            # Accelerometer data
            axes2[0].plot(df['timestamp_sec'].values, df['accel_x'].values,
                        label='X', alpha=0.7, linewidth=0.5, color='red')
            axes2[0].plot(df['timestamp_sec'].values, df['accel_y'].values,
                        label='Y', alpha=0.7, linewidth=0.5, color='green')
            axes2[0].plot(df['timestamp_sec'].values, df['accel_z'].values,
                        label='Z', alpha=0.7, linewidth=0.5, color='blue')
            axes2[0].set_ylabel('Acceleration (m/s²)')
            axes2[0].set_title('Accelerometer Data (±2g range)')
            axes2[0].legend(loc='upper right')
            axes2[0].grid(True, alpha=0.3)

            # Gyroscope data
            axes2[1].plot(df['timestamp_sec'].values, df['gyro_x'].values,
                        label='X', alpha=0.7, linewidth=0.5, color='red')
            axes2[1].plot(df['timestamp_sec'].values, df['gyro_y'].values,
                        label='Y', alpha=0.7, linewidth=0.5, color='green')
            axes2[1].plot(df['timestamp_sec'].values, df['gyro_z'].values,
                        label='Z', alpha=0.7, linewidth=0.5, color='blue')
            axes2[1].set_xlabel('Time (s)')
            axes2[1].set_ylabel('Angular Velocity (rad/s)')
            axes2[1].set_title('Gyroscope Data (±125 dps range)')
            axes2[1].legend(loc='upper right')
            axes2[1].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(sensor_plot_file, dpi=150)
            print(f"[INFO] Sensor plots saved: {sensor_plot_file}")
            plt.close(fig2)

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
