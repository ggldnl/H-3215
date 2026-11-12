# Tests and benchmarks

Run them from the repository folder, in the `hexapod` environment.

| Command | What it checks | Needs |
|---|---|---|
| `python tests/test_sts.py` | The servo driver, against simulated servos: packet bytes, ping and scan, state decoding, sync write and read, corrupted replies | Nothing |
| `python tests/servo_check.py` | Every servo's status and the bus timing, read-only | Servos connected |
| `python tests/servo_check.py --move` | A slow sine (±10°, 0.5 Hz) on every servo, with tracking error | Servos free to turn |
| `python tests/bench_loop.py --io-ms <ms>` | The 50 Hz control loop on a loaded Pi: wake-up delay, interval error, overruns | A Pi set up with `setup_pi.sh` |

## Control-loop benchmark

`bench_loop.py` runs a network the size of the rough-terrain policy (250 inputs: 63 from the robot's own sensors and a 187-point height scan; 512-256-128 hidden; 18 outputs) in a 50 Hz loop. It simulates the servo bus with a pause of `--io-ms` milliseconds: use the value `servo_check.py` prints. It runs three scenarios:

| Scenario | Background load | Loop |
|---|---|---|
| Idle | None | Default scheduling |
| Loaded | Matrix and memory work on the other cores | Default scheduling |
| Loaded + tuned | Same | Pinned to core 3, `SCHED_FIFO`, garbage collection frozen |

The tuned scenario is the one the robot uses: it needs the isolated core and the real-time permission from [`setup_pi.sh`](raspberry-pi.md).
