#!/usr/bin/env python3
"""
Phase 1 Data Analysis Script
Analyzes 500Hz IMU CSV data from LeRobot acquisition system
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def analyze_csv(csv_file):
    print("=" * 60)
    print("LeRobot Phase 1 Data Analysis")
    print("=" * 60)

    # Read CSV
    try:
        df = pd.read_csv(csv_file)
    except FileNotFoundError:
        print(f"[ERROR] File not found: {csv_file}")
        sys.exit(1)

    print(f"\n[INFO] Loaded {len(df)} samples from {csv_file}")

    # ========== BASIC STATS ==========
    print("\n" + "=" * 60)
    print("1. BASIC STATISTICS")
    print("=" * 60)

    duration = df['timestamp_sec'].max() - df['timestamp_sec'].min()
    actual_rate = len(df) / duration if duration > 0 else 0

    print(f"Duration: {duration:.2f} seconds")
    print(f"Actual sample rate: {actual_rate:.1f} Hz (target: 500 Hz)")
    print(f"Total samples: {len(df)}")

    # ========== DROPPED SAMPLES ==========
    print("\n" + "=" * 60)
    print("2. DATA INTEGRITY")
    print("=" * 60)

    # Check for gaps in generation counter
    generation_diffs = df['generation'].diff()
    drops = (generation_diffs > 1).sum()
    total_dropped = (generation_diffs[generation_diffs > 1] - 1).sum()

    print(f"Dropped samples: {int(total_dropped)}")
    print(f"Drop events: {drops}")

    if drops > 0:
        print(f"Drop locations (generation #):")
        drop_indices = df[generation_diffs > 1].index
        for idx in drop_indices[:10]:  # Show first 10
            gen = df.loc[idx, 'generation']
            prev_gen = df.loc[idx-1, 'generation']
            print(f"  Gap between gen {prev_gen} and {gen} (missed {gen - prev_gen - 1})")

    drop_rate = (total_dropped / (len(df) + total_dropped)) * 100 if len(df) > 0 else 0
    print(f"Drop rate: {drop_rate:.3f}%")

    # ========== TIMESTAMP JITTER ==========
    print("\n" + "=" * 60)
    print("3. TIMESTAMP JITTER ANALYSIS")
    print("=" * 60)

    df['dt_us'] = df['timestamp_sec'].diff() * 1e6  # Convert to microseconds

    target_period = 2000  # 500 Hz = 2000 µs period
    mean_dt = df['dt_us'].mean()
    std_dt = df['dt_us'].std()
    max_jitter_pos = df['dt_us'].max() - target_period
    max_jitter_neg = target_period - df['dt_us'].min()
    max_jitter = max(max_jitter_pos, max_jitter_neg)

    print(f"Mean interval: {mean_dt:.1f} µs (target: {target_period} µs)")
    print(f"Std deviation: {std_dt:.1f} µs")
    print(f"Max jitter: {max_jitter:.1f} µs")

    # Acceptance criteria
    jitter_pass = max_jitter < 200
    print(f"\nJitter < 200µs: {'✅ PASS' if jitter_pass else '❌ FAIL'}")

    # ========== IMU READ TIME ==========
    print("\n" + "=" * 60)
    print("4. IMU READ PERFORMANCE")
    print("=" * 60)

    print(f"IMU read time:")
    print(f"  Min: {df['imu_read_us'].min()} µs")
    print(f"  Max: {df['imu_read_us'].max()} µs")
    print(f"  Mean: {df['imu_read_us'].mean():.1f} µs")
    print(f"  Median: {df['imu_read_us'].median():.1f} µs")

    imu_budget = df['imu_read_us'].max() < 1800  # 90% of 2ms period
    print(f"\nIMU read < 1800µs: {'✅ PASS' if imu_budget else '❌ FAIL'}")

    # ========== SENSOR DATA RANGES ==========
    print("\n" + "=" * 60)
    print("5. SENSOR DATA RANGES")
    print("=" * 60)

    print("\nAccelerometer (m/s²):")
    print(f"  X: {df['accel_x'].min():.3f} to {df['accel_x'].max():.3f}")
    print(f"  Y: {df['accel_y'].min():.3f} to {df['accel_y'].max():.3f}")
    print(f"  Z: {df['accel_z'].min():.3f} to {df['accel_z'].max():.3f}")

    print("\nGyroscope (rad/s):")
    print(f"  X: {df['gyro_x'].min():.3f} to {df['gyro_x'].max():.3f}")
    print(f"  Y: {df['gyro_y'].min():.3f} to {df['gyro_y'].max():.3f}")
    print(f"  Z: {df['gyro_z'].min():.3f} to {df['gyro_z'].max():.3f}")

    # Sanity check - IMU should see gravity
    accel_magnitude = np.sqrt(df['accel_x']**2 + df['accel_y']**2 + df['accel_z']**2)
    print(f"\nAccel magnitude: {accel_magnitude.mean():.2f} ± {accel_magnitude.std():.2f} m/s²")
    print(f"Expected gravity: ~9.81 m/s²")

    # ========== OVERALL RESULT ==========
    print("\n" + "=" * 60)
    print("6. OVERALL RESULT")
    print("=" * 60)

    all_pass = (
        drop_rate < 0.1 and  # < 0.1% drops
        jitter_pass and
        imu_budget
    )

    if all_pass:
        print("✅ ALL TESTS PASSED - Phase 1 Validated!")
    else:
        print("❌ SOME TESTS FAILED - Review data")
        if drop_rate >= 0.1:
            print("  - High drop rate")
        if not jitter_pass:
            print("  - Excessive jitter")
        if not imu_budget:
            print("  - IMU read time too long")

    # ========== PLOT GENERATION ==========
    print("\n" + "=" * 60)
    print("7. GENERATING PLOTS")
    print("=" * 60)

    fig, axes = plt.subplots(4, 1, figsize=(14, 12))

    # Convert to numpy arrays for matplotlib compatibility
    time = df['timestamp_sec'].values
    dt_us = df['dt_us'].values

    # Plot 1: Timestamp Jitter
    axes[0].plot(time, dt_us, linewidth=0.5)
    axes[0].axhline(target_period, color='r', linestyle='--', label=f'Target ({target_period} µs)')
    axes[0].axhline(target_period + 200, color='orange', linestyle=':', label='Tolerance (±200 µs)')
    axes[0].axhline(target_period - 200, color='orange', linestyle=':')
    axes[0].set_ylabel('Inter-sample interval (µs)')
    axes[0].set_title('Timestamp Jitter @ 500Hz')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Plot 2: Accelerometer
    axes[1].plot(time, df['accel_x'].values, label='X', alpha=0.7, linewidth=0.8)
    axes[1].plot(time, df['accel_y'].values, label='Y', alpha=0.7, linewidth=0.8)
    axes[1].plot(time, df['accel_z'].values, label='Z', alpha=0.7, linewidth=0.8)
    axes[1].set_ylabel('Acceleration (m/s²)')
    axes[1].set_title('Accelerometer Data')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # Plot 3: Gyroscope
    axes[2].plot(time, df['gyro_x'].values, label='X', alpha=0.7, linewidth=0.8)
    axes[2].plot(time, df['gyro_y'].values, label='Y', alpha=0.7, linewidth=0.8)
    axes[2].plot(time, df['gyro_z'].values, label='Z', alpha=0.7, linewidth=0.8)
    axes[2].set_ylabel('Angular velocity (rad/s)')
    axes[2].set_title('Gyroscope Data')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    # Plot 4: IMU Read Time
    axes[3].plot(time, df['imu_read_us'].values, linewidth=0.5)
    axes[3].axhline(df['imu_read_us'].mean(), color='r', linestyle='--',
                    label=f'Mean ({df["imu_read_us"].mean():.0f} µs)')
    axes[3].set_xlabel('Time (s)')
    axes[3].set_ylabel('Read time (µs)')
    axes[3].set_title('IMU I2C Read Latency')
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()

    output_file = csv_file.replace('.csv', '_analysis.png')
    plt.savefig(output_file, dpi=150)
    print(f"[INFO] Saved plot to: {output_file}")

    # Also show if running interactively
    try:
        plt.show()
    except:
        pass  # No display available

    print("\n" + "=" * 60)
    print("Analysis complete!")
    print("=" * 60)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_phase1.py <csv_file>")
        print("Example: python3 analyze_phase1.py test_data.csv")
        sys.exit(1)

    analyze_csv(sys.argv[1])
