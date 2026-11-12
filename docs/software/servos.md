# Servos

The 18 STS3215 share one half-duplex bus at 1 Mbps. Each servo has an ID, and one sync write sets every target while one sync read returns every servo's position, speed, load, voltage and temperature.

Set the Waveshare board's jumpers to **B** (USB). The adapter then shows up as `/dev/ttyACM0`; check with `ls /dev/ttyACM* /dev/ttyUSB*`, and pass `--port` to the scripts if it differs.

## Setting up new servos

1. New servos all have ID 1. Connect **only** the new servo, then give it its ID:
   ```bash
   python scripts/servo_setup.py set-id 1 2
   ```
2. With all servos connected, remove the reply delay and list them:
   ```bash
   python scripts/servo_setup.py fast-reply
   ```
3. Check status and bus timing (read-only), then the motion test once the servos can turn freely:
   ```bash
   python tests/servo_check.py
   python tests/servo_check.py --move
   ```

!!! warning "Remove the reply delay"
    From the factory, each servo waits 500 µs before answering a read. With 18 servos that alone takes 9 ms of every 20 ms control cycle. `fast-reply` sets it to zero.

## Bus timing

`servo_check.py` measures a sync read with 1, 2, … servos, fits the time per servo, and projects a full cycle for 18: one write of every target plus one read of every state. It prints the number to pass to the control-loop benchmark as `--io-ms` (see [Tests and benchmarks](tests.md)).
