"""Checks the servo bus driver against simulated servos (no hardware needed).

    python tests/test_sts.py
"""
import struct

from hexapod import sts


class FakeServos:
    """Stands in for the serial port and answers like STS servos with the given memory."""

    def __init__(self, memory):
        self.memory = memory  # {id: bytearray}
        self.timeout, self.baudrate = 0.02, 1_000_000
        self.sent, self.rx = [], b""

    def reset_input_buffer(self):
        self.rx = b""

    def read(self, n):
        data, self.rx = self.rx[:n], self.rx[n:]
        return data

    def write(self, packet):
        assert packet[:2] == b"\xff\xff" and packet[-1] == ~sum(packet[2:-1]) & 0xFF
        self.sent.append(packet)
        servo_id, instruction, params = packet[2], packet[4], packet[5:-1]
        mem = self.memory.get(servo_id)
        if instruction == sts.PING and mem:
            self._reply(servo_id, b"")
        elif instruction == sts.READ and mem:
            address, size = params
            self._reply(servo_id, mem[address:address + size])
        elif instruction == sts.WRITE and mem:
            mem[params[0]:params[0] + len(params) - 1] = params[1:]
            self._reply(servo_id, b"")
            if params[0] == sts.ID:
                self.memory[params[1]] = self.memory.pop(servo_id)
        elif instruction == sts.SYNC_READ:
            address, size, *ids = params
            for i in ids:
                self._reply(i, self.memory[i][address:address + size])
        elif instruction == sts.SYNC_WRITE:
            address, size = params[:2]
            for k in range(2, len(params), size + 1):
                self.memory[params[k]][address:address + size] = params[k + 1:k + 1 + size]

    def _reply(self, servo_id, data):
        body = bytes([servo_id, len(data) + 2, 0]) + data
        self.rx += b"\xff\xff" + body + bytes([~sum(body) & 0xFF])


def make_bus():
    memory = {1: bytearray(70), 2: bytearray(70)}
    # position 2048, speed -100, load -50 (sign-magnitude), 12.0 V, 35 C
    memory[1][56:64] = struct.pack("<HHHBB", 2048, 0x8000 | 100, 0x400 | 50, 120, 35)
    bus = sts.Bus.__new__(sts.Bus)
    bus.ser = FakeServos(memory)
    return bus


def test_packet_bytes():
    bus = make_bus()
    bus.ping(1)
    assert bus.ser.sent[-1] == bytes.fromhex("ffff010201fb")  # checksum = ~(1 + 2 + 1)
    bus.read(1, sts.PRESENT_POSITION, 2)
    assert bus.ser.sent[-1] == bytes.fromhex("ffff0104023802be")


def test_ping_and_scan():
    bus = make_bus()
    assert bus.ping(1) and not bus.ping(3)
    assert bus.scan() == [1, 2]
    assert bus.ser.timeout == 0.02  # restored after the scan


def test_decode_state():
    bus = make_bus()
    assert sts.decode_state(bus.read(1, sts.PRESENT_POSITION, sts.STATE_SIZE)) == (2048, -100, -50, 12.0, 35)


def test_sync_write_then_sync_read():
    bus = make_bus()
    bus.sync_write(sts.GOAL_POSITION, {1: sts.u16(1000), 2: sts.u16(3000)})
    assert bus.sync_read([1, 2], sts.GOAL_POSITION, 2) == {1: sts.u16(1000), 2: sts.u16(3000)}


def test_write_and_corrupted_reply():
    bus = make_bus()
    bus.write(2, sts.TORQUE_ENABLE, b"\x01")
    assert bus.read(2, sts.TORQUE_ENABLE, 1) == b"\x01"
    bus._send(1, sts.PING)
    bus.ser.rx = bus.ser.rx[:-1] + b"\x00"  # flip the checksum
    try:
        bus._reply()
        raise AssertionError("corrupted reply accepted")
    except IOError:
        pass


if __name__ == "__main__":
    for name, test in list(globals().items()):
        if name.startswith("test_"):
            test()
    print("all tests passed")
