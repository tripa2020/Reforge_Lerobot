#!/usr/bin/env python3
"""
Phase 3 CSV Parser with Correct IMU-Encoder Misalignment Calculation

Key Insight: One-frame pipeline delay
- Frame k: IMU sampled, servo READ requested
- Frame k+1: Servo response arrives, contains encoder sample from ~frame k time

Therefore, to compute misalignment for frame k:
  Use servo_age and servo_latency from frame k+1

Misalignment formula:
  t_encoder ≈ t_imu - (FRAME_PERIOD - servo_age_next - servo_latency_next/2)

Simplified:
  misalignment ≈ FRAME_PERIOD - servo_age_next - servo_latency_next/2
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import struct
import sys
from pathlib import Path

# Phase 3 Configuration
FRAME_PERIOD_US = 2000.0  # 500 Hz = 2000 µs
MISALIGNMENT_THRESHOLD_US = 800.0  # Requirement: <800 µs

# SensorData structure (64 bytes)
SENSOR_DATA_FORMAT = (
    'I'   # frame_ts_us (4 bytes)
    'I'   # frame_index (4 bytes)
    'f'   # accel_x_mps2 (4 bytes)
    'f'   # accel_y_mps2 (4 bytes)
    'f'   # accel_z_mps2 (4 bytes)
    'f'   # gyro_x_rads (4 bytes)
    'f'   # gyro_y_rads (4 bytes)
    'f'   # gyro_z_rads (4 bytes)
    'i'   # servo_position (4 bytes)
    'i'   # servo_velocity (4 bytes)
    'H'   # imu_read_us (2 bytes)
    'H'   # servo_age_us (2 bytes)
    'H'   # isr_total_us (2 bytes)
    'H'   # servo_latency_us (2 bytes)
    'h'   # period_error_us (2 bytes)
    'I'   # checksum (4 bytes)
    'B'   # coherency_flag (1 byte)
    'B'   # servo_error_code (1 byte)
    '6s'  # reserved (6 bytes)
)

SENSOR_DATA_SIZE = struct.calcsize(SENSOR_DATA_FORMAT)
assert SENSOR_DATA_SIZE == 64, f"SensorData size mismatch: {SENSOR_DATA_SIZE} != 64"


def parse_binary_csv(filepath):
    """Parse Phase 3 binary CSV file (64-byte structs)."""
    with open(filepath, 'rb') as f:
        data = f.read()

    num_samples = len(data) // SENSOR_DATA_SIZE

    if len(data) % SENSOR_DATA_SIZE != 0:
        print(f"Warning: File size {len(data)} not multiple of {SENSOR_DATA_SIZE}")
        print(f"Truncating to {num_samples} complete samples")

    samples = []
    for i in range(num_samples):
        offset = i * SENSOR_DATA_SIZE
        chunk = data[offset:offset + SENSOR_DATA_SIZE]

        unpacked = struct.unpack(SENSOR_DATA_FORMAT, chunk)

        samples.append({
            'frame_ts_us': unpacked[0],
            'frame_index': unpacked[1],
            'accel_x_mps2': unpacked[2],
            'accel_y_mps2': unpacked[3],
            'accel_z_mps2': unpacked[4],
            'gyro_x_rads': unpacked[5],
            'gyro_y_rads': unpacked[6],
            'gyro_z_rads': unpacked[7],
            'servo_pos': unpacked[8],
            'servo_vel': unpacked[9],
            'imu_read_us': unpacked[10],
            'servo_age_us': unpacked[11],
            'isr_total_us': unpacked[12],
            'servo_latency_us': unpacked[13],
            'period_error_us': unpacked[14],
            'checksum': unpacked[15],
            'coherency_flag': unpacked[16],
            'servo_error_code': unpacked[17],
        })

    return pd.DataFrame(samples)


def add_aligned_timing(df):
    """
    Apply one-frame pipeline correction for IMU-encoder alignment.

    For frame k:
    - IMU was sampled at frame_ts_us[k]
    - Servo READ was requested
    - Servo response arrives by frame k+1

    Therefore:
    - servo_pos_aligned[k] = servo_pos[k+1] (encoder sample from ~frame k time)
    - Use servo_age_us[k+1] and servo_latency_us[k+1] to compute misalignment

    Misalignment formula:
      t_encoder ≈ t_imu - (FRAME_PERIOD - servo_age_next - servo_latency_next/2)

    Simplified (for magnitude):
      misalignment ≈ FRAME_PERIOD - servo_age_next - servo_latency_next/2
    """
    # Shift servo data from next frame (k+1) to align with IMU from current frame (k)
    df['servo_pos_aligned'] = df['servo_pos'].shift(-1)
    df['servo_age_next'] = df['servo_age_us'].shift(-1)
    df['servo_latency_next'] = df['servo_latency_us'].shift(-1)
    df['servo_error_next'] = df['servo_error_code'].shift(-1)

    # Drop last row (no "next" data available)
    df = df.iloc[:-1].reset_index(drop=True)

    # Calculate IMU-encoder misalignment (absolute magnitude)
    df['imu_servo_misalign_us'] = (
        FRAME_PERIOD_US
        - df['servo_age_next']
        - 0.5 * df['servo_latency_next']
    )

    # Mark valid samples (where servo had no errors)
    df['valid_alignment'] = (df['servo_error_next'] == 0) & (df['servo_pos_aligned'] >= 0)

    return df


def analyze_alignment(df):
    """Analyze IMU-encoder alignment quality."""
    valid = df[df['valid_alignment']]

    if len(valid) == 0:
        print("ERROR: No valid servo samples found!")
        return

    print("\n" + "="*60)
    print("IMU-ENCODER ALIGNMENT ANALYSIS")
    print("="*60)

    print(f"\nTotal frames: {len(df)}")
    print(f"Valid frames: {len(valid)} ({len(valid)/len(df)*100:.1f}%)")
    print(f"Invalid frames: {len(df) - len(valid)} ({(len(df)-len(valid))/len(df)*100:.1f}%)")

    print(f"\n--- IMU-Encoder Misalignment (µs) ---")
    print(f"Min:    {valid['imu_servo_misalign_us'].min():.1f}")
    print(f"Max:    {valid['imu_servo_misalign_us'].max():.1f}")
    print(f"Mean:   {valid['imu_servo_misalign_us'].mean():.1f}")
    print(f"Median: {valid['imu_servo_misalign_us'].median():.1f}")
    print(f"Std:    {valid['imu_servo_misalign_us'].std():.1f}")

    print(f"\n--- Servo Timing (Next Frame) ---")
    print(f"Servo age (µs):     {valid['servo_age_next'].mean():.1f} ± {valid['servo_age_next'].std():.1f}")
    print(f"Servo latency (µs): {valid['servo_latency_next'].mean():.1f} ± {valid['servo_latency_next'].std():.1f}")

    print(f"\n--- ISR Timing ---")
    print(f"IMU read (µs):  {valid['imu_read_us'].mean():.1f} ± {valid['imu_read_us'].std():.1f}")
    print(f"ISR total (µs): {valid['isr_total_us'].mean():.1f} ± {valid['isr_total_us'].std():.1f}")

    # Check against threshold
    over_threshold = valid[valid['imu_servo_misalign_us'] > MISALIGNMENT_THRESHOLD_US]

    print(f"\n--- Threshold Check ({MISALIGNMENT_THRESHOLD_US} µs) ---")
    if len(over_threshold) == 0:
        print(f"✅ PASS: All {len(valid)} frames < {MISALIGNMENT_THRESHOLD_US} µs")
        print(f"   Max misalignment: {valid['imu_servo_misalign_us'].max():.1f} µs")
        print(f"   Margin: {MISALIGNMENT_THRESHOLD_US - valid['imu_servo_misalign_us'].max():.1f} µs")
    else:
        print(f"❌ FAIL: {len(over_threshold)}/{len(valid)} frames > {MISALIGNMENT_THRESHOLD_US} µs")
        print(f"   Worst: {valid['imu_servo_misalign_us'].max():.1f} µs")
        print(f"   Overshoot: {valid['imu_servo_misalign_us'].max() - MISALIGNMENT_THRESHOLD_US:.1f} µs")

    print("="*60 + "\n")


def plot_alignment(df, output_path='IMU_encoder_misalignment.png'):
    """Generate comprehensive alignment visualization."""
    valid = df[df['valid_alignment']]

    if len(valid) == 0:
        print("ERROR: No valid data to plot!")
        return

    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('Phase 3: IMU-Encoder Alignment Analysis', fontsize=16, fontweight='bold')

    # Convert frame indices to time (seconds)
    time_s = valid['frame_index'] / 500.0  # 500 Hz

    # --- Plot 1: Misalignment Time Series ---
    ax = axes[0]
    ax.plot(time_s, valid['imu_servo_misalign_us'],
            linewidth=0.8, color='#2E86AB', alpha=0.8, label='Misalignment')
    ax.axhline(MISALIGNMENT_THRESHOLD_US, color='red', linestyle='--',
               linewidth=2, label=f'Threshold ({MISALIGNMENT_THRESHOLD_US} µs)')
    ax.axhline(valid['imu_servo_misalign_us'].mean(), color='orange',
               linestyle=':', linewidth=1.5, label=f'Mean ({valid["imu_servo_misalign_us"].mean():.1f} µs)')

    ax.fill_between(time_s, 0, valid['imu_servo_misalign_us'],
                     color='#2E86AB', alpha=0.2)

    ax.set_ylabel('Misalignment (µs)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_title('IMU-Encoder Temporal Misalignment\n' +
                 r'$\Delta t \approx T_{period} - servo\_age_{next} - 0.5 \cdot servo\_latency_{next}$',
                 fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_ylim(0, max(MISALIGNMENT_THRESHOLD_US * 1.2, valid['imu_servo_misalign_us'].max() * 1.1))

    # --- Plot 2: Histogram ---
    ax = axes[1]
    n, bins, patches = ax.hist(valid['imu_servo_misalign_us'], bins=50,
                                color='#A23B72', alpha=0.7, edgecolor='black')

    # Color bars above threshold red
    threshold_idx = np.searchsorted(bins, MISALIGNMENT_THRESHOLD_US)
    for i, patch in enumerate(patches):
        if bins[i] > MISALIGNMENT_THRESHOLD_US:
            patch.set_facecolor('red')
            patch.set_alpha(0.9)

    ax.axvline(MISALIGNMENT_THRESHOLD_US, color='red', linestyle='--',
               linewidth=2, label=f'Threshold ({MISALIGNMENT_THRESHOLD_US} µs)')
    ax.axvline(valid['imu_servo_misalign_us'].mean(), color='orange',
               linestyle=':', linewidth=2, label=f'Mean ({valid["imu_servo_misalign_us"].mean():.1f} µs)')
    ax.axvline(valid['imu_servo_misalign_us'].median(), color='green',
               linestyle='-.', linewidth=1.5, label=f'Median ({valid["imu_servo_misalign_us"].median():.1f} µs)')

    ax.set_xlabel('Misalignment (µs)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Count', fontsize=11)
    ax.set_title(f'Distribution (n={len(valid)})', fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, axis='y')

    # --- Plot 3: Servo Timing Components ---
    ax = axes[2]
    ax.plot(time_s, valid['servo_age_next'],
            linewidth=0.8, color='#F18F01', alpha=0.8, label='Servo Age (next)')
    ax.plot(time_s, valid['servo_latency_next'],
            linewidth=0.8, color='#6A4C93', alpha=0.8, label='Servo Latency (next)')
    ax.plot(time_s, valid['imu_read_us'],
            linewidth=0.8, color='#2D936C', alpha=0.8, label='IMU Read')

    ax.set_ylabel('Time (µs)', fontsize=12, fontweight='bold')
    ax.set_xlabel('Time (s)', fontsize=11)
    ax.set_title('Timing Components (One-Frame Pipeline Corrected)', fontsize=13)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, linestyle='--')

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"✅ Plot saved: {output_path}")
    plt.close()


def main():
    if len(sys.argv) < 2:
        print("Usage: python parse_phase3_csv.py <csv_file>")
        print("Example: python parse_phase3_csv.py phase3_data.bin")
        sys.exit(1)

    csv_path = Path(sys.argv[1])

    if not csv_path.exists():
        print(f"ERROR: File not found: {csv_path}")
        sys.exit(1)

    print(f"Parsing: {csv_path}")
    print(f"Expected struct size: {SENSOR_DATA_SIZE} bytes")

    # Parse binary CSV
    df = parse_binary_csv(csv_path)
    print(f"Loaded {len(df)} samples")

    # Apply one-frame pipeline correction
    df = add_aligned_timing(df)
    print(f"After alignment: {len(df)} samples (dropped last frame)")

    # Analyze
    analyze_alignment(df)

    # Plot
    output_png = csv_path.parent / 'IMU_encoder_misalignment.png'
    plot_alignment(df, output_png)

    # Save processed CSV
    output_csv = csv_path.parent / 'phase3_processed.csv'
    df.to_csv(output_csv, index=False)
    print(f"✅ Processed CSV saved: {output_csv}")


if __name__ == '__main__':
    main()
