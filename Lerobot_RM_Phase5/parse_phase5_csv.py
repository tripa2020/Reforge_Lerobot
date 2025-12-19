#!/usr/bin/env python3
"""
LeRobot Phase 4 CSV Parser with Integrated Analysis

Captures binary sensor data from Teensy 4.1 via Serial4 and automatically
runs timing/coherency analysis when capture completes.

Phase 4 Architecture (Option B' - Synchronized TX):
  - Frame ISR @ 500Hz: Read IMU → Queue servo TX → Snapshot latest_servo
  - UART3 ISR: Ring buffer RX/TX (non-blocking)
  - Main loop: servo_protocol_fsm() parses RX, updates latest_servo
  - One-cycle pipeline: "ask" in frame N, "know" in frame N+1

Key Timing Guarantee (Option B'):
  - t_frame = IMU sample time
  - t_req = t_frame + ~50µs (servo TX queued right after IMU read)
  - Servo encoder sample ≈ t_req + 200-400µs (internal delay)
  - Total misalignment ≈ 250-500µs (well under 800µs target)

Features:
  - Same handshake protocol (READY → START → ACK)
  - 64-byte Phase 4 struct format
  - Real-time sample counting with progress display
  - Automatic analysis on Ctrl+C or --duration timeout
  - Timing plots saved to PNG
  - Option B' servo age validation (<800µs)
  - Servo latency tracking (t_rx - t_req)

Usage:
  # Basic capture (Ctrl+C to stop, then auto-analyze)
  python3 parse_phase4_csv.py /dev/ttyACM0 2000000 -o test.csv -v

  # Timed capture (auto-stops after 60 seconds, then analyzes)
  python3 parse_phase4_csv.py /dev/ttyACM0 2000000 -o test.csv --duration 60

  # Skip analysis
  python3 parse_phase4_csv.py /dev/ttyACM0 2000000 -o test.csv --no-analyze

Date: 2025-12-07
Phase: 4 (continuing from Phase 3 with enhancements)
"""

import struct
import serial
import sys
import argparse
import signal
import time
import os

# ============== PHASE 4 DATA STRUCTURE (64 bytes) ==============
# Must match SensorData struct in Lerobot_RM_Phase4.ino
# Phase 4: Option B' - Synchronized TX Architecture

SAMPLE_SIZE = 64

# Struct format (little-endian):
# I: frame_ts_us (4 bytes) - ISR entry time (= IMU sample time)
# I: frame_index (4 bytes) - Monotonic counter (detect drops)
# 6f: accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z (24 bytes) - m/s² and rad/s
# 2i: servo_position, servo_velocity (8 bytes) - position 0-4095, velocity reserved
# 4H: imu_read_us, servo_age_us, isr_total_us, servo_latency_us (8 bytes)
#     servo_age_us = frame_ts_us - servo.t_rx_us
#     servo_latency_us = t_rx_us - t_req_us (round-trip time)
# h: period_error_us (2 bytes) - SIGNED jitter (actual period - 2000µs)
# 2x: padding for alignment (2 bytes)
# I: checksum (4 bytes) - XOR checksum for torn read detection
# B: coherency_flag (1 byte) - 0xAA=valid, 0xFF=torn, 0xEE=stale
# B: servo_error_code (1 byte) - 0=OK, 1=timeout, 2=checksum, 3=framing
# H: cmd_goal (2 bytes) - last commanded goal position (0xFFFF if none)
# I: cmd_time_us (4 bytes) - timestamp when command was sent
# Total: 4+4+24+8+8+2+2+4+1+1+2+4 = 64 bytes ✓

STRUCT_FMT = "<I I 6f 2i 4H h 2x I B B H I"
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

# Verify struct size matches expected
assert STRUCT_SIZE == SAMPLE_SIZE, f"Struct size mismatch: {STRUCT_SIZE} != {SAMPLE_SIZE}"

# Field names for CSV header
# Phase 4: servo_latency_us is now populated (was imu_servo_skew_us in Phase 2)
# Phase 4+: cmd_goal and cmd_time_us added for tracking error analysis
CSV_HEADER = (
    "timestamp_sec,frame_index,"
    "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,"
    "servo_pos,servo_vel,"
    "imu_read_us,servo_age_us,isr_total_us,servo_latency_us,"
    "period_error_us,checksum,coherency_flag,servo_error_code,"
    "cmd_goal,cmd_time_us"
)


def parse_sample(data):
    """
    Parse 64-byte binary sample into dict.
    Phase 4: Option B' - Synchronized TX Architecture
    """
    fields = struct.unpack(STRUCT_FMT, data)
    return {
        'timestamp_sec': fields[0] / 1e6,
        'frame_index': fields[1],
        'accel_x': fields[2],
        'accel_y': fields[3],
        'accel_z': fields[4],
        'gyro_x': fields[5],
        'gyro_y': fields[6],
        'gyro_z': fields[7],
        'servo_pos': fields[8],
        'servo_vel': fields[9],
        'imu_read_us': fields[10],
        'servo_age_us': fields[11],
        'isr_total_us': fields[12],
        'servo_latency_us': fields[13],  # t_rx - t_req (round-trip time)
        'period_error_us': fields[14],   # Signed!
        'checksum': fields[15],
        'coherency_flag': fields[16],
        'servo_error_code': fields[17],
        'cmd_goal': fields[18],          # Last commanded goal position
        'cmd_time_us': fields[19],       # Timestamp when command was sent
    }


def sample_to_csv_row(sample):
    """Convert sample dict to CSV row string."""
    return (
        f"{sample['timestamp_sec']:.6f},{sample['frame_index']},"
        f"{sample['accel_x']:.6f},{sample['accel_y']:.6f},{sample['accel_z']:.6f},"
        f"{sample['gyro_x']:.6f},{sample['gyro_y']:.6f},{sample['gyro_z']:.6f},"
        f"{sample['servo_pos']},{sample['servo_vel']},"
        f"{sample['imu_read_us']},{sample['servo_age_us']},"
        f"{sample['isr_total_us']},{sample['servo_latency_us']},"
        f"{sample['period_error_us']},{sample['checksum']},"
        f"{sample['coherency_flag']},{sample['servo_error_code']},"
        f"{sample['cmd_goal']},{sample['cmd_time_us']}"
    )


# ============== ONE-FRAME PIPELINE ALIGNMENT ==============

def align_servo_to_imu(df):
    """
    Fix one-frame pipeline delay: servo data in row k came from frame k-1.

    In Option B' architecture:
      - Frame ISR reads IMU at t_k, then queues servo TX
      - Servo response arrives before next frame
      - Frame k+1 snapshots the servo state (which is from the k request)

    This means each row's servo data is actually ONE CYCLE OLD relative to
    the IMU data in that same row. Naively treating "same row = same time"
    injects a systematic 2ms phase error into training data.

    Solution: Shift ALL servo data forward by 1 row to align with correct IMU frame.
    This includes position AND timing fields (servo_age_us, servo_latency_us).

    Key Insight:
      For frame k, to compute IMU-encoder misalignment:
        - IMU was sampled at frame_ts_us[k]
        - Servo READ was requested at ~frame_ts_us[k] + 50µs
        - Servo response arrives by frame k+1
        - Use servo_age_us[k+1] and servo_latency_us[k+1] to compute misalignment

    Misalignment formula:
      t_encoder ≈ t_imu - (FRAME_PERIOD - servo_age_next - servo_latency_next/2)

    Simplified (magnitude):
      misalignment ≈ FRAME_PERIOD - servo_age_next - servo_latency_next/2

    Args:
        df: DataFrame with servo fields

    Returns:
        DataFrame with aligned servo fields (_aligned suffix), last row dropped
    """
    # Shift ALL servo data from next frame (k+1) to align with IMU from current frame (k)
    df['servo_pos_aligned'] = df['servo_pos'].shift(-1)
    df['servo_age_aligned'] = df['servo_age_us'].shift(-1)
    df['servo_latency_aligned'] = df['servo_latency_us'].shift(-1)
    df['servo_error_aligned'] = df['servo_error_code'].shift(-1)

    # Drop last row (has NaN servo after shift)
    df = df.iloc[:-1].reset_index(drop=True)

    # Calculate IMU-encoder temporal misalignment using aligned timing
    FRAME_PERIOD_US = 2000.0  # 500 Hz
    df['imu_encoder_misalign_us'] = (
        FRAME_PERIOD_US
        - df['servo_age_aligned']
        - 0.5 * df['servo_latency_aligned']
    )

    # Mark valid alignment samples (servo had no errors and data is valid)
    df['valid_alignment'] = (
        (df['servo_error_aligned'] == 0) &
        (df['servo_pos_aligned'] >= 0) &
        (df['servo_age_aligned'] != 0xFFFF)
    )

    return df


# ============== INTEGRATED ANALYSIS ==============

def analyze_phase4(csv_file, save_plots=True):
    """
    Analyze Phase 4 (Option B') CSV data and print validation results.

    Option B' Architecture:
      - Frame ISR: IMU read → Queue servo TX → Snapshot latest_servo
      - Servo TX tightly synchronized with IMU read (<800µs misalignment)

    Optionally saves timing plots to PNG.
    """
    try:
        import pandas as pd
        import numpy as np
    except ImportError:
        print("[WARN] pandas/numpy not installed - skipping analysis", file=sys.stderr)
        print("[WARN] Install with: pip install pandas numpy matplotlib", file=sys.stderr)
        return None

    print("\n" + "=" * 70)
    print("LeRobot Phase 4 Validation Analysis")
    print("=" * 70)
    print("Architecture: Synchronized TX - Frame ISR queues servo TX after IMU read")
    print("Target: Encoder-IMU misalignment < 800µs")
    print("=" * 70)

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
    if len(df) > 1:
        df = df.iloc[1:].reset_index(drop=True)
        print(f"[INFO] Dropped row 0 (startup transient), analyzing {len(df)} samples")

    # ========== ONE-FRAME PIPELINE ALIGNMENT ==========
    # Fix: servo_pos in row k came from frame k-1 (systematic 2ms phase error)
    df = align_servo_to_imu(df)
    print(f"[INFO] Applied one-frame pipeline alignment, {len(df)} samples remaining")

    # ========== BASIC STATS ==========
    print("\n" + "-" * 70)
    print("1. BASIC STATISTICS")
    print("-" * 70)

    duration = df['timestamp_sec'].max() - df['timestamp_sec'].min()
    actual_rate = len(df) / duration if duration > 0 else 0

    print(f"Duration: {duration:.2f} seconds (from device timestamps)")
    print(f"Actual rate: {actual_rate:.1f} Hz (target: 500 Hz)")
    print(f"Total samples: {len(df)}")

    rate_pass = 490 <= actual_rate <= 510
    print(f"Rate within 490-510 Hz: {'✅ PASS' if rate_pass else '❌ FAIL'}")

    # ========== DATA INTEGRITY ==========
    print("\n" + "-" * 70)
    print("2. DATA INTEGRITY (Dropped Samples)")
    print("-" * 70)

    frame_diffs = df['frame_index'].diff()
    drops = (frame_diffs > 1).sum()
    total_dropped = int((frame_diffs[frame_diffs > 1] - 1).sum()) if drops > 0 else 0

    print(f"Dropped samples: {total_dropped}")
    print(f"Drop events: {drops}")
    drop_rate = (total_dropped / (len(df) + total_dropped)) * 100 if len(df) > 0 else 0
    print(f"Drop rate: {drop_rate:.4f}%")

    drop_pass = total_dropped == 0
    print(f"Zero drops: {'✅ PASS' if drop_pass else '❌ FAIL'}")

    # ========== JITTER ANALYSIS ==========
    print("\n" + "-" * 70)
    print("3. JITTER ANALYSIS (Period Error)")
    print("-" * 70)

    print(f"Mean: {df['period_error_us'].mean():.2f} µs")
    print(f"Std: {df['period_error_us'].std():.2f} µs")
    print(f"Max: {df['period_error_us'].max():.0f} µs")
    print(f"Min: {df['period_error_us'].min():.0f} µs")

    max_jitter = max(abs(df['period_error_us'].max()), abs(df['period_error_us'].min()))
    jitter_pass = max_jitter < 100
    print(f"Jitter < 100µs: {'✅ PASS' if jitter_pass else '❌ FAIL'}")

    # ========== WCET ANALYSIS ==========
    print("\n" + "-" * 70)
    print("4. WCET ANALYSIS (ISR Execution Time)")
    print("-" * 70)
    print("NOTE: Phase 4 ISR includes IMU read + servo TX queue (~100µs expected)")

    print(f"Max: {df['isr_total_us'].max()} µs")
    print(f"Mean: {df['isr_total_us'].mean():.1f} µs")
    print(f"Std: {df['isr_total_us'].std():.1f} µs")
    print(f"99.9th percentile: {df['isr_total_us'].quantile(0.999):.1f} µs")

    wcet_pass = df['isr_total_us'].max() < 200  # Tighter budget for Option B'
    print(f"WCET < 200µs: {'✅ PASS' if wcet_pass else '❌ FAIL'}")

    # ========== SERVO AGE ANALYSIS (Option B' - with pipeline correction) ==========
    print("\n" + "-" * 70)
    print("5. SERVO DATA AGE (Time Since Last Read, Pipeline-Corrected)")
    print("-" * 70)
    print("NOTE: servo_age_aligned = frame_ts - t_rx (time since servo response)")
    print("      One-frame pipeline: servo data shifted from next frame")
    print("      Expected: < 2000µs (one frame period @ 500 Hz)")

    # Filter to VALID servo samples only (using aligned fields)
    valid_servo = df[df['valid_alignment']]
    total_errors = (~df['valid_alignment']).sum()

    if len(valid_servo) > 0:
        print(f"\nValid servo samples: {len(valid_servo)} / {len(df)} ({len(valid_servo)/len(df)*100:.1f}%)")
        print(f"Invalid samples (excluded): {total_errors} ({total_errors/len(df)*100:.2f}%)")

        print(f"\nServo data age (time since last read, aligned):")
        print(f"  Mean: {valid_servo['servo_age_aligned'].mean():.1f} µs")
        print(f"  Std: {valid_servo['servo_age_aligned'].std():.1f} µs")
        print(f"  Min: {valid_servo['servo_age_aligned'].min():.0f} µs")
        print(f"  Max: {valid_servo['servo_age_aligned'].max():.0f} µs")
        print(f"  Median: {valid_servo['servo_age_aligned'].median():.1f} µs")

        # Check against 2000µs (one frame period)
        max_age = valid_servo['servo_age_aligned'].max()
        age_pass = max_age < 2000
        print(f"\nMax age < 2000µs: {'✅ PASS' if age_pass else '❌ FAIL'}")

        # Also check typical (median) age
        median_age = valid_servo['servo_age_aligned'].median()
        age_typical = median_age < 1800
        print(f"Median age < 1800µs: {'✅ PASS' if age_typical else '⚠️  HIGH (but may still work)'}")
    else:
        print("[WARN] No valid servo samples (all errors or invalid)")
        age_pass = False
        age_typical = False

    # ========== SERVO LATENCY ANALYSIS (NEW in Phase 4, pipeline-corrected) ==========
    print("\n" + "-" * 70)
    print("6. SERVO COMMUNICATION LATENCY (Round-Trip Time, Pipeline-Corrected)")
    print("-" * 70)
    print("NOTE: servo_latency_aligned = t_rx - t_req (READ request → response)")
    print("      Expected: < 380µs (theoretical sync write time)")
    print("      Lower is better - indicates fast servo response")

    valid_latency = df[df['valid_alignment'] & (df['servo_latency_aligned'] > 0) & (df['servo_latency_aligned'] < 2000)]

    if len(valid_latency) > 0:
        print(f"\nValid latency samples: {len(valid_latency)} / {len(df)}")
        print(f"\nServo round-trip latency (READ request → response):")
        print(f"  Mean: {valid_latency['servo_latency_aligned'].mean():.1f} µs")
        print(f"  Std: {valid_latency['servo_latency_aligned'].std():.1f} µs")
        print(f"  Min: {valid_latency['servo_latency_aligned'].min():.0f} µs")
        print(f"  Max: {valid_latency['servo_latency_aligned'].max():.0f} µs")
        print(f"  Median: {valid_latency['servo_latency_aligned'].median():.1f} µs")

        # Check against 380µs (theoretical communication time)
        mean_latency = valid_latency['servo_latency_aligned'].mean()
        max_latency = valid_latency['servo_latency_aligned'].max()
        latency_excellent = mean_latency < 200
        latency_pass = max_latency < 380

        if latency_excellent:
            print(f"\n✅ EXCELLENT: Mean latency {mean_latency:.1f}µs < 200µs (very fast!)")
        elif latency_pass:
            print(f"\n✅ PASS: Max latency {max_latency:.0f}µs < 380µs")
        else:
            print(f"\n⚠️  HIGH: Max latency {max_latency:.0f}µs > 380µs")

        latency_reasonable = latency_pass
    else:
        print("[WARN] No valid latency data")
        latency_reasonable = True  # Don't fail on missing data

    # ========== COHERENCY (TORN READS) ==========
    print("\n" + "-" * 70)
    print("7. COHERENCY (Seqlock Torn Reads)")
    print("-" * 70)

    torn = (df['coherency_flag'] == 0xFF).sum()
    stale = (df['coherency_flag'] == 0xEE).sum()
    valid = (df['coherency_flag'] == 0xAA).sum()

    print(f"Valid samples (0xAA): {valid}")
    print(f"Torn reads (0xFF): {torn}")
    print(f"Stale reads (0xEE): {stale}")

    coherency_pass = torn == 0
    print(f"\nZero torn reads: {'✅ PASS' if coherency_pass else '❌ FAIL'}")

    # ========== IMU-ENCODER MISALIGNMENT (Option B' Key Metric) ==========
    print("\n" + "-" * 70)
    print("7. IMU-ENCODER TEMPORAL MISALIGNMENT (Pipeline-Corrected)")
    print("-" * 70)
    print("NOTE: Misalignment = FRAME_PERIOD - servo_age_aligned - 0.5*servo_latency_aligned")
    print("      Target: < 800µs (LeRobot requirement)")

    valid_misalign = df[df['valid_alignment']]

    if len(valid_misalign) > 0:
        print(f"\nValid alignment samples: {len(valid_misalign)} / {len(df)}")
        print(f"\nIMU-Encoder Misalignment:")
        print(f"  Mean: {valid_misalign['imu_encoder_misalign_us'].mean():.1f} µs")
        print(f"  Std: {valid_misalign['imu_encoder_misalign_us'].std():.1f} µs")
        print(f"  Min: {valid_misalign['imu_encoder_misalign_us'].min():.1f} µs")
        print(f"  Max: {valid_misalign['imu_encoder_misalign_us'].max():.1f} µs")
        print(f"  Median: {valid_misalign['imu_encoder_misalign_us'].median():.1f} µs")

        # Critical check: < 800µs
        max_misalign = valid_misalign['imu_encoder_misalign_us'].max()
        misalign_pass = max_misalign < 800
        over_threshold = valid_misalign[valid_misalign['imu_encoder_misalign_us'] > 800]

        if misalign_pass:
            print(f"\n✅ PASS: All {len(valid_misalign)} frames < 800µs")
            print(f"   Max misalignment: {max_misalign:.1f} µs")
            print(f"   Margin: {800 - max_misalign:.1f} µs")
        else:
            print(f"\n❌ FAIL: {len(over_threshold)}/{len(valid_misalign)} frames > 800µs")
            print(f"   Worst: {max_misalign:.1f} µs")
            print(f"   Overshoot: {max_misalign - 800:.1f} µs")
    else:
        print("[WARN] No valid alignment data")
        misalign_pass = False

    # ========== SERVO ERRORS ==========
    print("\n" + "-" * 70)
    print("8. SERVO COMMUNICATION")
    print("-" * 70)

    # Use aligned error field for accuracy
    servo_errors = (~df['valid_alignment']).sum()
    # For detailed breakdown, use original error codes (not shifted)
    servo_timeouts = (df['servo_error_code'] == 1).sum()
    servo_checksum = (df['servo_error_code'] == 2).sum()
    servo_framing = (df['servo_error_code'] == 3).sum()

    print(f"Total errors/invalid: {servo_errors} / {len(df)} ({servo_errors/len(df)*100:.2f}%)")
    print(f"  Timeouts (1): {servo_timeouts}")
    print(f"  Checksum (2): {servo_checksum}")
    print(f"  Framing (3): {servo_framing}")

    servo_pass = (servo_errors / len(df)) < 0.05  # < 5% error rate
    print(f"Servo errors < 5%: {'✅ PASS' if servo_pass else '❌ FAIL'}")

    # ========== IMU READ TIME ==========
    print("\n" + "-" * 70)
    print("9. IMU READ TIME (Bare-Metal Driver)")
    print("-" * 70)

    print(f"IMU read time (SPI burst, 14 bytes):")
    print(f"  Mean: {df['imu_read_us'].mean():.1f} µs")
    print(f"  Max: {df['imu_read_us'].max()} µs")
    print(f"  Min: {df['imu_read_us'].min()} µs")
    print(f"  Std: {df['imu_read_us'].std():.1f} µs")

    imu_fast = df['imu_read_us'].mean() < 100
    print(f"Mean IMU read < 100µs: {'✅ PASS' if imu_fast else '❌ FAIL'}")

    # ========== OVERALL RESULT ==========
    print("\n" + "=" * 70)
    print("PHASE 4 VALIDATION SUMMARY")
    print("=" * 70)

    all_pass = (rate_pass and drop_pass and jitter_pass and wcet_pass and
                age_pass and misalign_pass and coherency_pass and servo_pass and imu_fast)

    results = {
        'Sample rate 490-510 Hz': rate_pass,
        'Zero drops': drop_pass,
        'Jitter < 100µs': jitter_pass,
        'WCET < 200µs': wcet_pass,
        'Servo data age < 2000µs': age_pass,
        'IMU-encoder misalign < 800µs': misalign_pass,
        'Zero torn reads': coherency_pass,
        'Servo errors < 5%': servo_pass,
        'IMU read < 100µs': imu_fast,
    }

    for name, passed in results.items():
        status = '✅ PASS' if passed else '❌ FAIL'
        print(f"  {name}: {status}")

    print("\n" + "=" * 70)
    if all_pass:
        print("✅ ALL TESTS PASSED - Phase 4 Validated!")
        print("   Synchronized TX architecture achieving <800µs encoder-IMU alignment!")
    else:
        print("❌ SOME TESTS FAILED - Review issues above")
    print("=" * 70)

    # ========== GENERATE PLOTS ==========
    if save_plots:
        try:
            import matplotlib.pyplot as plt

            # ===== TIMING/DIAGNOSTIC PLOTS (5 subplots) =====
            plot_file = csv_file.replace('.csv', '_analysis.png')
            fig1, axes1 = plt.subplots(5, 1, figsize=(16, 15))

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
            axes1[1].axhline(200, color='r', linestyle='--', label='Budget (200µs)')
            axes1[1].set_ylabel('ISR Time (µs)')
            axes1[1].set_title('ISR Execution Time (WCET) - Option B\' Target: <200µs')
            axes1[1].legend()
            axes1[1].grid(True, alpha=0.3)

            # Servo Age (Option B' - pipeline-corrected)
            valid_servo_plot = df[df['valid_alignment']].copy()
            if len(valid_servo_plot) > 0:
                axes1[2].plot(valid_servo_plot['timestamp_sec'].values,
                            valid_servo_plot['servo_age_aligned'].values,
                            linewidth=0.5, alpha=0.7, color='#2E86AB')
                axes1[2].axhline(2000, color='r', linestyle='--', label='Frame period (2000µs)')
                axes1[2].axhline(valid_servo_plot['servo_age_aligned'].median(), color='g',
                               linestyle='--', label=f'Median ({valid_servo_plot["servo_age_aligned"].median():.0f}µs)')
                axes1[2].set_ylabel('Data Age (µs)')
                axes1[2].set_title('Servo Data Age - Time Since Last Read (Pipeline-Corrected)')
                axes1[2].legend()
                axes1[2].grid(True, alpha=0.3)
            else:
                axes1[2].text(0.5, 0.5, 'No valid servo data', ha='center', va='center',
                            transform=axes1[2].transAxes)
                axes1[2].set_title('Servo Data Age (No Valid Data)')

            # Servo Latency (NEW in Phase 4, pipeline-corrected)
            valid_lat_plot = df[df['valid_alignment'] & (df['servo_latency_aligned'] > 0) & (df['servo_latency_aligned'] < 2000)].copy()
            if len(valid_lat_plot) > 0:
                axes1[3].plot(valid_lat_plot['timestamp_sec'].values,
                            valid_lat_plot['servo_latency_aligned'].values,
                            linewidth=0.5, alpha=0.7, color='purple')
                axes1[3].axhline(380, color='orange', linestyle='--', label='Theory (380µs)')
                axes1[3].axhline(valid_lat_plot['servo_latency_aligned'].mean(), color='g',
                               linestyle='--', label=f'Mean ({valid_lat_plot["servo_latency_aligned"].mean():.0f}µs)')
                axes1[3].set_ylabel('Latency (µs)')
                axes1[3].set_title('Servo Communication Latency (READ request → response)')
                axes1[3].legend()
                axes1[3].grid(True, alpha=0.3)
            else:
                axes1[3].text(0.5, 0.5, 'No valid latency data', ha='center', va='center',
                            transform=axes1[3].transAxes)
                axes1[3].set_title('Servo Latency (No Valid Data)')

            # IMU read time
            axes1[4].plot(df['timestamp_sec'].values, df['imu_read_us'].values,
                        alpha=0.7, linewidth=0.5, color='blue')
            axes1[4].set_xlabel('Time (s)')
            axes1[4].set_ylabel('Read Time (µs)')
            axes1[4].set_title('IMU Read Time (Bare-Metal Driver)')
            axes1[4].axhline(df['imu_read_us'].mean(), color='g', linestyle='--',
                           label=f'Mean ({df["imu_read_us"].mean():.1f} µs)')
            axes1[4].legend()
            axes1[4].grid(True, alpha=0.3)

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

            # ===== IMU-ENCODER MISALIGNMENT PLOT (3 subplots) =====
            misalign_plot_file = csv_file.replace('.csv', '_IMU_encoder_misalignment.png')
            valid_misalign_plot = df[df['valid_alignment']].copy()

            if len(valid_misalign_plot) > 0:
                fig3, axes3 = plt.subplots(3, 1, figsize=(14, 10))
                fig3.suptitle('Phase 4: IMU-Encoder Alignment Analysis (Pipeline-Corrected)',
                             fontsize=16, fontweight='bold')

                # Convert frame indices to time (seconds)
                time_s = valid_misalign_plot['timestamp_sec'].values

                # --- Plot 1: Misalignment Time Series ---
                ax = axes3[0]
                ax.plot(time_s, valid_misalign_plot['imu_encoder_misalign_us'].values,
                       linewidth=0.8, color='#2E86AB', alpha=0.8, label='Misalignment')
                ax.axhline(800, color='red', linestyle='--',
                          linewidth=2, label='Threshold (800 µs)')
                ax.axhline(valid_misalign_plot['imu_encoder_misalign_us'].mean(), color='orange',
                          linestyle=':', linewidth=1.5,
                          label=f'Mean ({valid_misalign_plot["imu_encoder_misalign_us"].mean():.1f} µs)')

                ax.fill_between(time_s, 0, valid_misalign_plot['imu_encoder_misalign_us'].values,
                               color='#2E86AB', alpha=0.2)

                ax.set_ylabel('Misalignment (µs)', fontsize=12, fontweight='bold')
                ax.set_xlabel('Time (s)', fontsize=11)
                ax.set_title('IMU-Encoder Temporal Misalignment\n' +
                            r'$\Delta t \approx T_{period} - servo\_age_{aligned} - 0.5 \cdot servo\_latency_{aligned}$',
                            fontsize=13)
                ax.legend(loc='upper right', fontsize=10)
                ax.grid(True, alpha=0.3, linestyle='--')
                ax.set_ylim(0, max(800 * 1.2, valid_misalign_plot['imu_encoder_misalign_us'].max() * 1.1))

                # --- Plot 2: Histogram ---
                ax = axes3[1]
                n, bins, patches = ax.hist(valid_misalign_plot['imu_encoder_misalign_us'].values,
                                          bins=50, color='#A23B72', alpha=0.7, edgecolor='black')

                # Color bars above threshold red
                for i, patch in enumerate(patches):
                    if bins[i] > 800:
                        patch.set_facecolor('red')
                        patch.set_alpha(0.9)

                ax.axvline(800, color='red', linestyle='--',
                          linewidth=2, label='Threshold (800 µs)')
                ax.axvline(valid_misalign_plot['imu_encoder_misalign_us'].mean(), color='orange',
                          linestyle=':', linewidth=2,
                          label=f'Mean ({valid_misalign_plot["imu_encoder_misalign_us"].mean():.1f} µs)')
                ax.axvline(valid_misalign_plot['imu_encoder_misalign_us'].median(), color='green',
                          linestyle='-.', linewidth=1.5,
                          label=f'Median ({valid_misalign_plot["imu_encoder_misalign_us"].median():.1f} µs)')

                ax.set_xlabel('Misalignment (µs)', fontsize=12, fontweight='bold')
                ax.set_ylabel('Count', fontsize=11)
                ax.set_title(f'Distribution (n={len(valid_misalign_plot)})', fontsize=13)
                ax.legend(loc='upper right', fontsize=10)
                ax.grid(True, alpha=0.3, axis='y')

                # --- Plot 3: Timing Components ---
                ax = axes3[2]
                ax.plot(time_s, valid_misalign_plot['servo_age_aligned'].values,
                       linewidth=0.8, color='#F18F01', alpha=0.8, label='Servo Age (aligned)')
                ax.plot(time_s, valid_misalign_plot['servo_latency_aligned'].values,
                       linewidth=0.8, color='#6A4C93', alpha=0.8, label='Servo Latency (aligned)')
                ax.plot(time_s, valid_misalign_plot['imu_read_us'].values,
                       linewidth=0.8, color='#2D936C', alpha=0.8, label='IMU Read')

                ax.set_ylabel('Time (µs)', fontsize=12, fontweight='bold')
                ax.set_xlabel('Time (s)', fontsize=11)
                ax.set_title('Timing Components (One-Frame Pipeline Corrected)', fontsize=13)
                ax.legend(loc='upper right', fontsize=10)
                ax.grid(True, alpha=0.3, linestyle='--')

                plt.tight_layout()
                plt.savefig(misalign_plot_file, dpi=150, bbox_inches='tight')
                print(f"[INFO] IMU-encoder misalignment plot saved: {misalign_plot_file}")
                plt.close(fig3)
            else:
                print("[WARN] No valid alignment data - skipping misalignment plot", file=sys.stderr)

            # ===== SERVO CONTROL PLOT (2 subplots) =====
            # Plot commanded vs actual position for trajectory tracking analysis
            servo_control_plot_file = csv_file.replace('.csv', '_servo_control.png')
            valid_control = df[df['valid_alignment']].copy()

            # Filter to frames where commands were sent (cmd_goal != 0xFFFF)
            valid_control = valid_control[valid_control['cmd_goal'] != 0xFFFF]

            if len(valid_control) > 10:
                fig4, axes4 = plt.subplots(2, 1, figsize=(16, 10))
                fig4.suptitle('Phase 4: Servo Control Analysis (Command Tracking)',
                             fontsize=16, fontweight='bold')

                # Convert to time (seconds)
                time_s = valid_control['timestamp_sec'].values

                # --- Plot 1: Position vs Time (Commanded and Actual) ---
                ax = axes4[0]
                ax.plot(time_s, valid_control['cmd_goal'].values,
                       linewidth=1.2, color='#E63946', alpha=0.8, label='Commanded Position',
                       linestyle='--', marker='o', markersize=2, markevery=max(1, len(time_s)//100))
                ax.plot(time_s, valid_control['servo_pos_aligned'].values,
                       linewidth=1.0, color='#2A9D8F', alpha=0.9, label='Actual Position (Aligned)')

                ax.set_ylabel('Position (encoder units)', fontsize=12, fontweight='bold')
                ax.set_xlabel('Time (s)', fontsize=11)
                ax.set_title('Commanded vs Actual Servo Position', fontsize=13)
                ax.legend(loc='upper right', fontsize=11)
                ax.grid(True, alpha=0.3, linestyle='--')

                # Add range info to title
                pos_min = valid_control['servo_pos_aligned'].min()
                pos_max = valid_control['servo_pos_aligned'].max()
                pos_range = pos_max - pos_min
                ax.text(0.02, 0.98, f'Range: {pos_min:.0f}-{pos_max:.0f} ({pos_range:.0f} units)',
                       transform=ax.transAxes, fontsize=10,
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

                # --- Plot 2: Tracking Error (Commanded - Actual) ---
                ax = axes4[1]
                tracking_error = valid_control['cmd_goal'].values - valid_control['servo_pos_aligned'].values

                ax.plot(time_s, tracking_error,
                       linewidth=0.8, color='#F77F00', alpha=0.8, label='Tracking Error')
                ax.axhline(0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)
                ax.axhline(tracking_error.mean(), color='red', linestyle='--',
                          linewidth=1.5, label=f'Mean Error ({tracking_error.mean():.1f} units)')

                # Fill positive/negative error regions
                ax.fill_between(time_s, 0, tracking_error,
                               where=(tracking_error >= 0), color='#E63946', alpha=0.3,
                               label='Positive Error', interpolate=True)
                ax.fill_between(time_s, 0, tracking_error,
                               where=(tracking_error < 0), color='#2A9D8F', alpha=0.3,
                               label='Negative Error', interpolate=True)

                ax.set_ylabel('Error (units)', fontsize=12, fontweight='bold')
                ax.set_xlabel('Time (s)', fontsize=11)
                ax.set_title('Tracking Error (Commanded - Actual)', fontsize=13)
                ax.legend(loc='upper right', fontsize=10)
                ax.grid(True, alpha=0.3, linestyle='--')

                # Add error statistics to plot
                error_stats = (f'RMS: {np.sqrt(np.mean(tracking_error**2)):.1f} | '
                              f'Max: {tracking_error.max():.1f} | '
                              f'Min: {tracking_error.min():.1f} | '
                              f'Std: {tracking_error.std():.1f}')
                ax.text(0.02, 0.98, error_stats,
                       transform=ax.transAxes, fontsize=10,
                       verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))

                plt.tight_layout()
                plt.savefig(servo_control_plot_file, dpi=150, bbox_inches='tight')
                print(f"[INFO] Servo control plot saved: {servo_control_plot_file}")
                plt.close(fig4)

                # Print tracking error summary
                print("\n" + "-" * 70)
                print("8. SERVO CONTROL ANALYSIS (Command Tracking)")
                print("-" * 70)
                print(f"Samples with commands: {len(valid_control)}")
                print(f"Tracking error (commanded - actual):")
                print(f"  Mean: {tracking_error.mean():.2f} encoder units")
                print(f"  RMS: {np.sqrt(np.mean(tracking_error**2)):.2f} encoder units")
                print(f"  Std: {tracking_error.std():.2f} encoder units")
                print(f"  Max: {tracking_error.max():.1f} encoder units")
                print(f"  Min: {tracking_error.min():.1f} encoder units")
            else:
                print("[WARN] Not enough samples with commands - skipping servo control plot", file=sys.stderr)

        except ImportError:
            print("[WARN] matplotlib not installed - skipping plots", file=sys.stderr)
        except Exception as e:
            print(f"[WARN] Failed to generate plots: {e}", file=sys.stderr)

    return results


# ============== MAIN CAPTURE FUNCTION ==============

def main():
    parser = argparse.ArgumentParser(
        description="LeRobot Phase 4 CSV Parser with Integrated Analysis",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic capture (Ctrl+C to stop, then auto-analyze)
  python3 parse_phase4_csv.py /dev/ttyACM0 2000000 -o test.csv -v

  # Timed capture (auto-stops after 60 seconds)
  python3 parse_phase4_csv.py /dev/ttyACM0 2000000 -o test.csv --duration 60

  # Skip analysis
  python3 parse_phase4_csv.py /dev/ttyACM0 2000000 -o test.csv --no-analyze
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
    handshake_timeout = 30
    start_time = time.time()

    while not handshake_complete:
        if time.time() - start_time > handshake_timeout:
            print(f"[ERROR] Handshake timeout - no READY received", file=sys.stderr)
            print(f"[ERROR] Check Teensy USB connection and upload", file=sys.stderr)
            ser.close()
            outfile.close()
            sys.exit(1)

        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('ascii').strip()

                if line == "READY":
                    if args.verbose:
                        print(f"[INFO] Received READY from Teensy", file=sys.stderr)
                        print(f"[INFO] Sending START command...", file=sys.stderr)

                    ser.write(b"START\n")
                    ser.flush()

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
                pass

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
        if args.duration > 0 and (time.time() - capture_start) > args.duration:
            if args.verbose:
                print(f"\n[INFO] Duration reached ({args.duration}s)", file=sys.stderr)
            break

        waiting = ser.in_waiting
        if waiting > 0:
            buffer += ser.read(waiting)
        else:
            buffer += ser.read(1)

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
        analyze_phase4(args.output, save_plots=not args.no_plots)


if __name__ == "__main__":
    main()
