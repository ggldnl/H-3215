"""Minimal driver for Feetech STS servos (STS3215) on a serial bus.

Request: FF FF <id> <length> <instruction> <params...> <checksum>
Reply:   FF FF <id> <length> <error flags> <params...> <checksum>
length = number of params + 2, checksum = ~(sum of the bytes after FF FF) & 0xFF.
Multi-byte values are little-endian.
"""
import struct

import serial

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST_ID = 0xFE

# STS3215 register addresses. EEPROM registers (below 40) keep their value without power.
MODEL = 3              # u16, 777 for an STS3215
ID = 5                 # u8
RETURN_DELAY = 7       # u8, wait before replying, in 2 us steps
TORQUE_ENABLE = 40     # u8
GOAL_POSITION = 42     # u16
LOCK = 55              # u8, 0 = EEPROM writable
PRESENT_POSITION = 56  # start of the state block, see decode_state()
STATE_SIZE = 8
STATUS = 65            # u8 error flags, 0 = no error

STEPS_PER_TURN = 4096  # positions go 0-4095 over 360 degrees


class Bus:
    def __init__(self, port, baudrate=1_000_000, timeout=0.02):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def ping(self, servo_id):
        self._send(servo_id, PING)
        try:
            return self._reply()[0] == servo_id
        except OSError:
            return False

    def scan(self):
        """IDs of every servo that answers."""
        timeout, self.ser.timeout = self.ser.timeout, 0.005  # most IDs are absent: don't wait long
        try:
            return [i for i in range(BROADCAST_ID) if self.ping(i)]
        finally:
            self.ser.timeout = timeout

    def read(self, servo_id, address, size):
        self._send(servo_id, READ, bytes([address, size]))
        return self._reply()[2]

    def write(self, servo_id, address, data):
        self._send(servo_id, WRITE, bytes([address]) + data)
        self._reply()  # servos confirm every write (factory response level)

    def sync_read(self, ids, address, size):
        """The same registers from several servos with one request: {id: bytes}."""
        self._send(BROADCAST_ID, SYNC_READ, bytes([address, size, *ids]))
        return {servo_id: data for servo_id, _, data in (self._reply() for _ in ids)}

    def sync_write(self, address, data):
        """data = {id: bytes}, the same length for every servo. Servos don't reply."""
        size = len(next(iter(data.values())))
        params = bytes([address, size]) + b"".join(bytes([i]) + d for i, d in data.items())
        self._send(BROADCAST_ID, SYNC_WRITE, params)

    def _send(self, servo_id, instruction, params=b""):
        body = bytes([servo_id, len(params) + 2, instruction]) + params
        self.ser.reset_input_buffer()  # drop leftovers of an earlier failed exchange
        self.ser.write(b"\xff\xff" + body + bytes([~sum(body) & 0xFF]))

    def _reply(self):
        """One status packet: (id, error flags, params)."""
        head = self.ser.read(4)  # FF FF id length
        if len(head) < 4 or head[:2] != b"\xff\xff" or head[3] < 2:
            raise TimeoutError("no reply from the servo")
        rest = self.ser.read(head[3])  # error flags, params, checksum
        if len(rest) < head[3] or (~sum(head[2:] + rest[:-1]) & 0xFF) != rest[-1]:
            raise IOError("corrupted reply")
        return head[2], rest[0], rest[1:-1]


def u16(value):
    return struct.pack("<H", value)


def decode_state(data):
    """State block -> (position steps, speed steps/s, load per mille, volts, degrees C)."""
    position, speed, load, voltage, temperature = struct.unpack("<HHHBB", data)
    return position, _signed(speed, 15), _signed(load, 10), voltage / 10, temperature


def _signed(value, sign_bit):
    """Feetech stores signed values as a magnitude plus one sign bit."""
    magnitude = value & ((1 << sign_bit) - 1)
    return -magnitude if value >> sign_bit & 1 else magnitude
