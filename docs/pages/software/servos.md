# Servos

The 18 STS3215 share one half-duplex bus at 1 Mbps. Each servo has an ID, and one sync write sets every target while one sync read returns every servo's position, speed, load, voltage and temperature.

Set the Waveshare board's jumpers to **B** (USB). The adapter then shows up as `/dev/ttyACM0`; check with `ls /dev/ttyACM* /dev/ttyUSB*`, and pass `--port` to `servo_tool` if it differs.

## Setting up new servos

Each servo's ID follows the joint order: leg 1 joint 1 (coxa) is ID 1, leg 1 joint 2 is 2, leg 1 joint 3 is 3, leg 2 joint 1 is 4, and so on to leg 6 joint 3, ID 18. The ROS description takes them from the model, see [ROS 2](ros.md#servo-ids-and-calibration).

`servo_tool`, in `h3215_hardware`, does it all, with the same protocol code as the control loop. It needs the bus to itself: stop the robot's launch files first.

1. New servos all have ID 1. Connect **only** the new servo, then give it its ID:
   ```bash
   ros2 run h3215_hardware servo_tool set-id 1 2
   ```
2. With all servos connected, remove the reply delay and list them:
   ```bash
   ros2 run h3215_hardware servo_tool fast-reply
   ```
3. Check status and bus timing (read-only), then the motion test once the servos can turn freely:
   ```bash
   ros2 run h3215_hardware servo_tool check
   ros2 run h3215_hardware servo_tool check --move
   ```

| Command | Does |
|---|---|
| `scan` | Lists every servo: ID, model, reply delay, position |
| `set-id OLD NEW` | Gives a servo a new ID, refusing one that is taken |
| `fast-reply` | Sets every servo's reply delay to zero |
| `check` | Status of every servo (position, speed, load, voltage, temperature, reply delay, error flags) and the bus timing. `--ids` limits it to some servos |
| `check --move` | Then a slow sine on every servo, with the bus cycle time and the tracking error. `--amplitude` (10°), `--rate` (50 Hz), `--seconds` (10). Ctrl+C brings the servos back to where they started and turns torque off |

!!! warning "Remove the reply delay"
    From the factory, each servo waits 500 µs before answering a read. With 18 servos that alone takes 9 ms of every 20 ms control cycle. `fast-reply` sets it to zero.

## Bus timing

`servo_tool check` measures a sync read with 1, 2, … servos, fits the time per servo, and projects a full cycle for 18: one write of every target plus one read of every state. That is the time the control loop spends on the bus every cycle, out of 10 ms at 100 Hz.
