#!/usr/bin/env python3
import argparse
import time
from dataclasses import dataclass
from typing import Optional, List

import serial
from serial.tools import list_ports


TEENSY_VID = 0x16C0  # common PJRC VID (may vary by USB type / OS driver)


def pick_default_port() -> Optional[str]:
    ports = list(list_ports.comports())
    if not ports:
        return None

    # Prefer anything that looks like a Teensy / PJRC / ACM
    for p in ports:
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "teensy" in desc or "pjrc" in desc or "usb serial" in desc:
            return p.device
        if p.vid == TEENSY_VID:
            return p.device
        if "acm" in p.device.lower():  # /dev/ttyACM*
            return p.device

    # Fallback: first port
    return ports[0].device


def open_serial(port: str, baud: int) -> serial.Serial:
    # timeout matters; readline() can block forever without it. 
    ser = serial.Serial(port, baudrate=baud, timeout=0.2, write_timeout=0.2)
    # give Teensy/OS a moment to settle
    time.sleep(0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send_line(ser: serial.Serial, line: str) -> None:
    if not line.endswith("\n"):
        line += "\n"
    ser.write(line.encode("ascii", errors="strict"))


def read_lines_for(ser: serial.Serial, seconds: float) -> List[str]:
    t0 = time.time()
    out: List[str] = []
    while time.time() - t0 < seconds:
        b = ser.readline()
        if not b:
            continue
        try:
            s = b.decode("ascii", errors="replace").strip()
        except Exception:
            s = repr(b)
        if s:
            out.append(s)
    return out


def cmd_expect_prefix(ser: serial.Serial, cmd: str, prefix: str, timeout_s: float = 1.0) -> str:
    send_line(ser, cmd)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        b = ser.readline()
        if not b:
            continue
        s = b.decode("ascii", errors="replace").strip()
        if s.startswith(prefix):
            return s
    raise TimeoutError(f"Timeout waiting for prefix '{prefix}' after cmd '{cmd}'")


def cmd_expect_contains(ser: serial.Serial, cmd: str, substr: str, timeout_s: float = 1.0) -> str:
    send_line(ser, cmd)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        b = ser.readline()
        if not b:
            continue
        s = b.decode("ascii", errors="replace").strip()
        if substr in s:
            return s
    raise TimeoutError(f"Timeout waiting for '{substr}' after cmd '{cmd}'")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="e.g. /dev/ttyACM0 or COM7")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--cycles", type=int, default=20)
    ap.add_argument("--verbose", action="store_true")

    args = ap.parse_args()
    port = args.port or pick_default_port()
    if not port:
        raise SystemExit("No serial ports found.")

    print(f"[INFO] Using port={port} baud={args.baud}")
    ser = open_serial(port, args.baud)

    # handshake: keep poking until Teensy responds
    # Teensy USB serial can deliver bursts (whole packets) rather than trickling bytes. 
    ok = False
    for _ in range(20):
        try:
            r = cmd_expect_prefix(ser, "PING", "PONG", timeout_s=0.4)
            print(f"[OK] {r}")
            ok = True
            break
        except TimeoutError:
            time.sleep(0.1)
    if not ok:
        print("[WARN] No PONG. Dumping any boot output seen:")
        for s in read_lines_for(ser, 0.8):
            print("  ", s)

    # explicit mode transitions
    for i in range(args.cycles):
        r1 = cmd_expect_prefix(ser, "MODE,MOVE", "MODE,MOVE", timeout_s=1.0)
        r2 = cmd_expect_prefix(ser, "MODE?", "MODE,", timeout_s=1.0)
        if args.verbose:
            print(f"[{i:02d}] {r1} | {r2}")

        r3 = cmd_expect_prefix(ser, "MODE,DATA", "MODE,DATA", timeout_s=1.0)
        r4 = cmd_expect_prefix(ser, "MODE?", "MODE,", timeout_s=1.0)
        if args.verbose:
            print(f"[{i:02d}] {r3} | {r4}")

    print("[DONE] Mode transition test completed.")
    ser.close()


if __name__ == "__main__":
    main()
