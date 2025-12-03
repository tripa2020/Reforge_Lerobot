#!/usr/bin/env python3
"""
Binary CSV parser for LeRobot Phase 1 data.

Usage:
    python3 parse_phase1_csv.py /dev/ttyUSB0 921600 --output data.csv
    python3 parse_phase1_csv.py COM5 921600 -o data.csv  # Windows

Press Ctrl+C to stop capture.
"""

import struct
import serial
import sys
import argparse
import signal
import time
from dataclasses import dataclass

# CSVSample struct format (64 bytes, little-endian)
# uint32_t timestamp_us
# float accel_x, accel_y, accel_z
# float gyro_x, gyro_y, gyro_z
# int32_t servo_pos, servo_vel                 (Phase 2)
# uint32_t generation
# uint16_t imu_read_us                          (SPI read duration)
# uint16_t servo_read_us                        (Phase 1: SPI timing validation)
# uint8_t reserved[20]

SAMPLE_SIZE = 64
STRUCT_FMT = "<I 6f 2i I 2H 20x"  # Little-endian, 64 bytes total

@dataclass
class CSVSample:
    timestamp_us: int
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    servo_pos: int
    servo_vel: int
    generation: int
    imu_read_us: int
    servo_read_us: int

    @classmethod
    def from_bytes(cls, data: bytes):
        unpacked = struct.unpack(STRUCT_FMT, data)
        return cls(*unpacked)

    def to_csv_row(self) -> str:
        t_sec = self.timestamp_us / 1e6
        return (f"{t_sec:.6f},{self.generation},"
                f"{self.accel_x:.6f},{self.accel_y:.6f},{self.accel_z:.6f},"
                f"{self.gyro_x:.6f},{self.gyro_y:.6f},{self.gyro_z:.6f},"
                f"{self.servo_pos},{self.servo_vel},"
                f"{self.imu_read_us},{self.servo_read_us}")

def csv_header() -> str:
    return ("timestamp_sec,generation,"
            "accel_x,accel_y,accel_z,"
            "gyro_x,gyro_y,gyro_z,"
            "servo_pos,servo_vel,"
            "imu_read_us,servo_read_us")

def main():
    parser = argparse.ArgumentParser(description="Parse LeRobot binary CSV stream")
    parser.add_argument("port", help="Serial port (e.g., /dev/ttyUSB0, COM5)")
    parser.add_argument("baud", type=int, help="Baud rate (e.g., 921600)")
    parser.add_argument("-o", "--output", help="Output CSV file (default: stdout)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Print progress to stderr")
    args = parser.parse_args()

    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    if args.verbose:
        print(f"[INFO] Opened {args.port} @ {args.baud} baud", file=sys.stderr)
        print(f"[INFO] Press Ctrl+C to stop", file=sys.stderr)

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
                        sys.exit(1)

            except UnicodeDecodeError:
                # Ignore non-ASCII bytes during handshake
                pass

    # Clear any remaining handshake data
    ser.reset_input_buffer()

    # Open output file
    if args.output:
        outfile = open(args.output, 'w')
    else:
        outfile = sys.stdout

    # Write CSV header
    print(csv_header(), file=outfile)

    # Setup signal handler for clean exit
    sample_count = 0

    def signal_handler(sig, frame):
        if args.verbose:
            print(f"\n[INFO] Capture stopped. Total samples: {sample_count}", file=sys.stderr)
        ser.close()
        if args.output:
            outfile.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Parse loop
    buffer = b""

    while True:
        # Read available data
        waiting = ser.in_waiting
        if waiting > 0:
            buffer += ser.read(waiting)
        else:
            buffer += ser.read(1)  # Block for at least 1 byte

        # Process complete samples
        while len(buffer) >= SAMPLE_SIZE:
            chunk = buffer[:SAMPLE_SIZE]
            buffer = buffer[SAMPLE_SIZE:]

            try:
                sample = CSVSample.from_bytes(chunk)
                print(sample.to_csv_row(), file=outfile)
                sample_count += 1

                if args.verbose and sample_count % 1000 == 0:
                    print(f"[INFO] {sample_count} samples", file=sys.stderr)

            except struct.error as e:
                if args.verbose:
                    print(f"[WARN] Parse error: {e}", file=sys.stderr)
                continue

if __name__ == "__main__":
    main()
