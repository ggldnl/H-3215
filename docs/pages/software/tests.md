# Tests and benchmarks

Run them from the repository folder.

| Command | What it checks | Needs |
|---|---|---|
| `colcon test` in the workspace | `h3215_hardware`: the servo protocol and bus, against simulated servos on a pseudo-terminal (packet bytes, ping, read, sync write and read, a missing servo, corrupted replies). `h3215_core`: the battery estimate and the posture moves | ROS 2 Jazzy, or its Docker image (see [ROS 2](ros.md#building-on-a-pc)) |
| `uv run --project h3215/software/training python tests/test_morphology.py` | The model parser on H-3215 and on models with 4 to 8 legs and 2 to 4 joints each, its errors for names that break the convention, and that the generated ROS description is up to date | MuJoCo: a PC |
| `ros2 run h3215_hardware servo_tool check` | Every servo's status and the bus timing, read-only | Servos connected |
| `ros2 run h3215_hardware servo_tool check --move` | A slow sine (±10°, 0.5 Hz) on every servo, with tracking error | Servos free to turn |
| `python3 tests/bench_loop.py --io-ms <ms>` | A 50 Hz loop in Python on a loaded Pi: wake-up delay, interval error, overruns | A Pi set up with `setup_pi.sh`, `python3-numpy` |

## Control-loop benchmark

`bench_loop.py` runs a network the size of the rough-terrain policy (250 inputs: 63 from the robot's own sensors and a 187-point height scan; 512-256-128 hidden; 18 outputs) in a 50 Hz loop. It simulates the servo bus with a pause of `--io-ms` milliseconds: use the projected cycle `servo_tool check` prints. It runs three scenarios:

| Scenario | Background load | Loop |
|---|---|---|
| Idle | None | Default scheduling |
| Loaded | Matrix and memory work on the other cores | Default scheduling |
| Loaded + tuned | Same | Pinned to core 3, `SCHED_FIFO`, garbage collection frozen |

The tuned scenario needs the isolated core and the real-time permission from [`setup_pi.sh`](raspberry-pi.md).

!!! note "The robot's loop is no longer Python"
    `bench_loop.py` was written when the control loop was going to be a Python program. The loop now runs in C++, in ros2_control's controller manager, so this benchmark shows how much a Python loop would suffer, not how the robot's loop behaves. That needs a benchmark of the controller manager itself, on the Pi.
