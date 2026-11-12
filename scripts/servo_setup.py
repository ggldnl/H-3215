#!/usr/bin/env python3
"""One-time servo configuration, stored in each servo's EEPROM.

    python scripts/servo_setup.py scan             list the servos and their settings
    python scripts/servo_setup.py set-id 1 2       give the servo with ID 1 the ID 2
    python scripts/servo_setup.py fast-reply       make every servo reply without delay

New servos all ship with ID 1: connect only the new servo when you change its ID,
otherwise every servo with ID 1 answers at once.
If `scan` shows a reply delay above 0, run fast-reply: at 500 us per servo, 18 servos
would add 9 ms to every read.
"""
import argparse
import sys

from hexapod import sts


def write_eeprom(bus, servo_id, address, value):
    """EEPROM is write-protected: unlock, write, lock again."""
    bus.write(servo_id, sts.LOCK, b"\x00")
    bus.write(servo_id, address, bytes([value]))
    bus.write(value if address == sts.ID else servo_id, sts.LOCK, b"\x01")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0", help="servo adapter serial port")
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("scan")
    set_id = sub.add_parser("set-id")
    set_id.add_argument("old", type=int)
    set_id.add_argument("new", type=int)
    sub.add_parser("fast-reply")
    args = ap.parse_args()

    bus = sts.Bus(args.port)
    if args.command == "set-id":
        if not 0 <= args.new < sts.BROADCAST_ID:
            sys.exit(f"IDs go from 0 to {sts.BROADCAST_ID - 1}")
        if not bus.ping(args.old):
            sys.exit(f"No servo answers as ID {args.old}")
        if bus.ping(args.new):
            sys.exit(f"ID {args.new} is already taken")
        write_eeprom(bus, args.old, sts.ID, args.new)

    ids = bus.scan()
    if args.command == "fast-reply":
        for i in ids:
            if bus.read(i, sts.RETURN_DELAY, 1)[0]:
                write_eeprom(bus, i, sts.RETURN_DELAY, 0)

    if not ids:
        sys.exit("No servos found: check the servo power supply, the cable, the port (--port), "
                 "and that the board's jumpers are on B (USB).")
    print(" id  model  reply delay  position")
    for i in ids:
        model = int.from_bytes(bus.read(i, sts.MODEL, 2), "little")
        delay_us = bus.read(i, sts.RETURN_DELAY, 1)[0] * 2
        position = int.from_bytes(bus.read(i, sts.PRESENT_POSITION, 2), "little")
        print(f"{i:>3}  {model:>5}  {delay_us:>8} us  {position * 360 / sts.STEPS_PER_TURN:6.1f} deg")


if __name__ == "__main__":
    main()
