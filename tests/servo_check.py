#!/usr/bin/env python3
"""Checks the servos on the bus and measures how long bus I/O takes.

    python tests/servo_check.py           status and bus timing (read-only, nothing moves)
    python tests/servo_check.py --move    then a slow sine (+/-10 deg, 0.5 Hz) on every servo

The timing projects one control cycle (sync-write goals + sync-read state) to 18 servos:
pass that number to tests/bench_loop.py as --io-ms.
Before --move, make sure every servo can turn freely by the amplitude.
"""
import argparse
import math
import time

import numpy as np

from hexapod import sts

BITS_PER_BYTE = 10  # 8 data bits + start bit + stop bit


def status(bus, ids):
    print(" id  position   speed    load  voltage  temp  reply delay  errors")
    for i in ids:
        pos, speed, load, volt, temp = sts.decode_state(bus.read(i, sts.PRESENT_POSITION, sts.STATE_SIZE))
        delay_us = bus.read(i, sts.RETURN_DELAY, 1)[0] * 2
        errors = bus.read(i, sts.STATUS, 1)[0]
        print(f"{i:>3}  {pos * 360 / sts.STEPS_PER_TURN:5.1f} deg  {speed:6d}  {load / 10:5.1f} %  "
              f"{volt:5.1f} V  {temp:2d} C  {delay_us:8d} us    0x{errors:02x}")


def sync_read_ms(bus, ids, n=300):
    t = np.empty(n)
    for k in range(n):
        t0 = time.perf_counter()
        bus.sync_read(ids, sts.PRESENT_POSITION, sts.STATE_SIZE)
        t[k] = time.perf_counter() - t0
    return t * 1000


def timing(bus, ids):
    counts, medians = range(1, len(ids) + 1), []
    for k in counts:
        t = sync_read_ms(bus, ids[:k])
        medians.append(np.median(t))
        print(f"  sync-read, {k} servo(s): median {medians[-1]:.2f}  p99 {np.percentile(t, 99):.2f}  "
              f"max {t.max():.2f} ms")
    # Read time = fixed cost + cost per servo. With one servo, count it all as per servo.
    per_servo, fixed = np.polyfit(counts, medians, 1) if len(ids) > 1 else (medians[0], 0.0)
    read_18 = fixed + 18 * per_servo
    write_18 = (8 + 18 * 3) * BITS_PER_BYTE / bus.ser.baudrate * 1000  # 18 goal positions, no reply
    print(f"  per servo {per_servo:.3f} ms, fixed {fixed:.3f} ms")
    print(f"  projected 18-servo cycle: {write_18:.2f} write + {read_18:.2f} read = "
          f"{write_18 + read_18:.2f} ms  ->  tests/bench_loop.py --io-ms {write_18 + read_18:.1f}")


def move(bus, ids, amplitude_deg, rate, seconds):
    """Slow sine around the current positions; reports bus time and tracking error."""
    state = bus.sync_read(ids, sts.PRESENT_POSITION, sts.STATE_SIZE)
    start = {i: sts.decode_state(d)[0] for i, d in state.items()}
    amplitude = amplitude_deg * sts.STEPS_PER_TURN / 360
    bus.sync_write(sts.GOAL_POSITION, {i: sts.u16(p) for i, p in start.items()})  # goal = here...
    bus.sync_write(sts.TORQUE_ENABLE, {i: b"\x01" for i in ids})                 # ...then power on
    n = int(seconds * rate)
    io_ms, error_deg = np.empty(n), np.empty(n)
    try:
        next_cycle = time.perf_counter()
        for k in range(n):
            offset = amplitude * math.sin(2 * math.pi * 0.5 * k / rate)
            goal = {i: min(max(round(p + offset), 0), sts.STEPS_PER_TURN - 1) for i, p in start.items()}
            t0 = time.perf_counter()
            bus.sync_write(sts.GOAL_POSITION, {i: sts.u16(g) for i, g in goal.items()})
            state = bus.sync_read(ids, sts.PRESENT_POSITION, sts.STATE_SIZE)
            io_ms[k] = (time.perf_counter() - t0) * 1000
            error_deg[k] = max(abs(sts.decode_state(state[i])[0] - goal[i]) for i in ids) * 360 / sts.STEPS_PER_TURN
            next_cycle += 1 / rate
            time.sleep(max(0.0, next_cycle - time.perf_counter()))
    finally:  # also on Ctrl+C: back to the start, then let go
        bus.sync_write(sts.GOAL_POSITION, {i: sts.u16(p) for i, p in start.items()})
        time.sleep(1.0)
        bus.sync_write(sts.TORQUE_ENABLE, {i: b"\x00" for i in ids})
    print(f"  bus cycle (write + read): median {np.median(io_ms):.2f}  p99 {np.percentile(io_ms, 99):.2f}  "
          f"max {io_ms.max():.2f} ms")
    print(f"  tracking error (includes the servo's lag): median {np.median(error_deg):.2f}  "
          f"max {error_deg.max():.2f} deg")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="servo adapter serial port")
    ap.add_argument("--ids", type=int, nargs="+", help="servo IDs (default: scan the bus)")
    ap.add_argument("--move", action="store_true", help="also run the sine motion test")
    ap.add_argument("--amplitude", type=float, default=10, help="sine amplitude in degrees (default 10)")
    ap.add_argument("--rate", type=float, default=50, help="control rate for --move in Hz (default 50)")
    ap.add_argument("--seconds", type=float, default=10, help="duration of --move (default 10)")
    args = ap.parse_args()

    bus = sts.Bus(args.port)
    ids = args.ids or bus.scan()
    if not ids:
        raise SystemExit("No servos found: run scripts/servo_setup.py scan to troubleshoot.")
    print(f"Servos on {args.port}: {ids}\n")
    status(bus, ids)
    print("\nBus timing")
    timing(bus, ids)
    if args.move:
        print(f"\nSine motion, +/-{args.amplitude:g} deg at 0.5 Hz, {args.rate:g} Hz control, {args.seconds:g} s")
        move(bus, ids, args.amplitude, args.rate, args.seconds)


if __name__ == "__main__":
    main()
