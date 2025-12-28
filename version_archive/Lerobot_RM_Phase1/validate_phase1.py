#!/usr/bin/env python3
"""
Phase 1 Validation Script

Analyzes CSV data to verify Phase 1 acceptance criteria:
  ✓ Mean interval: 2000 ± 5 µs
  ✓ Max jitter: < 200 µs
  ✓ Dropped samples: 0
  ✓ Sample rate: 500 ± 0.5 Hz

Usage:
    python3 validate_phase1.py test_30s.csv
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import sys

def validate(csv_file):
    print("=" * 50)
    print("PHASE 1 VALIDATION")
    print("=" * 50)

    df = pd.read_csv(csv_file)

    # Basic stats
    duration = df['timestamp_sec'].max() - df['timestamp_sec'].min()
    sample_count = len(df)
    actual_rate = sample_count / duration

    print(f"\n--- Basic Metrics ---")
    print(f"Total samples: {sample_count}")
    print(f"Duration: {duration:.2f} seconds")
    print(f"Actual rate: {actual_rate:.2f} Hz")

    # Timestamp analysis
    df['dt_us'] = df['timestamp_sec'].diff() * 1e6

    mean_dt = df['dt_us'].mean()
    std_dt = df['dt_us'].std()
    max_jitter = max(abs(df['dt_us'].max() - 2000), abs(2000 - df['dt_us'].min()))

    print(f"\n--- Jitter Analysis ---")
    print(f"Mean interval: {mean_dt:.2f} µs (target: 2000 µs)")
    print(f"Std deviation: {std_dt:.2f} µs")
    print(f"Max jitter: {max_jitter:.2f} µs")

    # Acceptance criteria
    rate_ok = abs(actual_rate - 500) < 0.5
    mean_ok = abs(mean_dt - 2000) < 5
    jitter_ok = max_jitter < 200

    print(f"\n--- Pass/Fail Criteria ---")
    print(f"Sample rate (500 ± 0.5 Hz): {'✅ PASS' if rate_ok else '❌ FAIL'} ({actual_rate:.2f} Hz)")
    print(f"Mean interval (2000 ± 5 µs): {'✅ PASS' if mean_ok else '❌ FAIL'} ({mean_dt:.2f} µs)")
    print(f"Jitter (< 200 µs): {'✅ PASS' if jitter_ok else '❌ FAIL'} ({max_jitter:.2f} µs)")

    # Dropped samples
    gaps = (df['generation'].diff() != 1).sum()
    drops_ok = gaps == 0

    print(f"\n--- Data Integrity ---")
    print(f"Dropped samples: {gaps}")
    print(f"Zero drops: {'✅ PASS' if drops_ok else '❌ FAIL'}")

    # IMU read time
    print(f"\n--- IMU Performance ---")
    print(f"I2C read time: min={df['imu_read_us'].min()} µs, "
          f"max={df['imu_read_us'].max()} µs, "
          f"mean={df['imu_read_us'].mean():.1f} µs")

    imu_budget_ok = df['imu_read_us'].max() < 1800  # Should be << 2ms

    print(f"I2C budget (< 1800 µs): {'✅ PASS' if imu_budget_ok else '❌ FAIL'}")

    # Overall
    all_pass = rate_ok and mean_ok and jitter_ok and drops_ok and imu_budget_ok

    print(f"\n{'=' * 50}")
    if all_pass:
        print("✅ ALL TESTS PASSED - Phase 1 Complete!")
    else:
        print("❌ SOME TESTS FAILED - Review Issues")
    print("=" * 50)

    # Generate plots
    print(f"\nGenerating plots...")

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))

    # Plot 1: Jitter
    axes[0].plot(df['timestamp_sec'], df['dt_us'], linewidth=0.5, alpha=0.7)
    axes[0].axhline(2000, color='r', linestyle='--', linewidth=2, label='Target (2000 µs)')
    axes[0].axhline(2000 + 200, color='orange', linestyle=':', linewidth=1.5, label='Tolerance (±200 µs)')
    axes[0].axhline(2000 - 200, color='orange', linestyle=':', linewidth=1.5)
    axes[0].set_ylabel('Inter-sample interval (µs)', fontsize=12)
    axes[0].set_title('Phase 1: Timestamp Jitter @ 500Hz', fontsize=14, fontweight='bold')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim([1800, 2200])

    # Plot 2: I2C Read Time
    axes[1].plot(df['timestamp_sec'], df['imu_read_us'], linewidth=0.5, alpha=0.7, color='green')
    axes[1].axhline(df['imu_read_us'].mean(), color='blue', linestyle='--',
                    linewidth=2, label=f'Mean ({df["imu_read_us"].mean():.1f} µs)')
    axes[1].set_ylabel('I2C Read Time (µs)', fontsize=12)
    axes[1].set_title('IMU I2C Transaction Duration', fontsize=14, fontweight='bold')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # Plot 3: Accelerometer data (sanity check)
    axes[2].plot(df['timestamp_sec'], df['accel_x'], label='Accel X', alpha=0.7, linewidth=0.8)
    axes[2].plot(df['timestamp_sec'], df['accel_y'], label='Accel Y', alpha=0.7, linewidth=0.8)
    axes[2].plot(df['timestamp_sec'], df['accel_z'], label='Accel Z', alpha=0.7, linewidth=0.8)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].set_ylabel('Acceleration (m/s²)', fontsize=12)
    axes[2].set_title('IMU Accelerometer Data', fontsize=14, fontweight='bold')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()

    output_file = csv_file.replace('.csv', '_validation.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Plots saved to: {output_file}")

    # Show plots
    plt.show()

    return all_pass

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 validate_phase1.py <csv_file>")
        print("Example: python3 validate_phase1.py test_30s.csv")
        sys.exit(1)

    csv_file = sys.argv[1]

    try:
        success = validate(csv_file)
        sys.exit(0 if success else 1)
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
