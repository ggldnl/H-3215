# Software

Everything about the robot is in `h3215/`, with the hardware next to the software:

| Folder | Contents | Runs on |
|---|---|---|
| `h3215/hardware/h3215_description/` | The robot model, as a ROS package: the MJCF, the meshes, the URDF generated from the MJCF (see [Hardware](../hardware/index.md#robot-model)) | |
| `h3215/software/control/` | The low-level control loop, in C++: `h3215_hardware`, the ros2_control system that reads and writes the servo bus, and `servo_tool`, which sets up and checks the servos. The policy's controller will go here too | Pi |
| `h3215/software/core/` | The high-level interface, in Python: `h3215_core`, the ROS nodes above the control loop (`posture`, `monitor`), and `h3215_bringup`, the launch files for the control loop, the camera and the whole robot | Pi |
| `h3215/software/training/` | Everything that needs MuJoCo: `h3215_train`, with the model tools (`morphology`, the robot's structure read from its MJCF; `description`, the URDF generated from it) and the mjlab training tasks | PC |

Next to `h3215/`:

| Folder | Contents |
|---|---|
| `scripts/` | Setup you run once on the Pi: `setup_pi.sh`, `setup_ros.sh` |
| `tests/` | `test_morphology.py` (the model tools, on a PC) and `bench_loop.py` |

## Two environments

| Machine | What | Installed by |
|---|---|---|
| Raspberry Pi | ROS 2 and the four ROS packages, built in `~/h3215_ws` | `bash scripts/setup_ros.sh`, with every dependency from the `package.xml` files through rosdep |
| PC | `h3215_train`, with MuJoCo, mjlab and PyTorch | `uv sync --project h3215/software/training`, from `h3215/software/training/uv.lock` |

The Pi has no Python environment of its own: its Python code is ROS nodes, which use the system's Python and ROS 2's packages. A PC can also build the ROS packages, in Docker (see [ROS 2](ros.md#building-on-a-pc)).

## The ROS workspace

The ROS packages live in the repository, but the workspace that builds them doesn't. `scripts/setup_ros.sh` creates `~/h3215_ws` and links the repository into its `src/` folder. colcon finds `h3215_description`, `h3215_hardware`, `h3215_core` and `h3215_bringup` wherever they sit, and the build output stays out of the repository. See [ROS 2](ros.md).

## One model, read by everything

`h3215/hardware/h3215_description/mjcf/h3215.xml` describes the robot once. Training reads it through `h3215_train.morphology`, and the URDF is generated from it. Both find the legs, joints and feet from their names, so the model can change (more legs, more joints per leg) without touching the code. The naming convention is in [ROS 2](ros.md#the-description-comes-from-the-model) and at the top of the model.

## Training and the robot meet in one file

Every checkpoint the training saves is also exported to ONNX, with the joint order, PD gains, default pose, observation layout and action scale stored in its metadata. The robot loads that file and has everything it needs to run the policy, so the Pi never needs mjlab, PyTorch or CUDA.
