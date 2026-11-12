# Software

Everything that runs on the robot or trains its policy lives in `hexapod/`, as two separate Python projects:

| Path | Contents | Runs on | Environment |
|---|---|---|---|
| `hexapod/src/hexapod/` | The robot package, importable as `hexapod`. So far: `sts.py`, the servo bus driver | Raspberry Pi | conda env `hexapod`, from `hexapod/environment.yml` |
| `hexapod/train/` | The mjlab project: velocity-tracking tasks and PPO settings | A PC with an NVIDIA GPU | uv, from `hexapod/train/uv.lock` |

Next to them, at the root of the repository:

| Path | Contents |
|---|---|
| `scripts/` | Setup you run once: `setup_pi.sh`, `setup_env.sh`, `servo_setup.py` |
| `tests/` | Checks and benchmarks: `test_sts.py`, `servo_check.py`, `bench_loop.py` |

## Two projects, one interface

The robot code never imports the training code. They meet in a single file: every checkpoint the training saves is also exported to ONNX, with the joint order, PD gains, default pose, observation layout and action scale stored in its metadata. The robot loads that file and has everything it needs to run the policy.

This keeps the Pi free of mjlab, PyTorch and CUDA, and lets each side change its environment on its own.

!!! info "The Pi environment will change"
    The robot side will connect to ROS 2. Its Python environment depends on which operating system the Pi runs, which is still being decided, so `hexapod/environment.yml` is likely to be replaced.
