# ROS 2

The robot runs ROS 2 Jazzy on Ubuntu 24.04. The joints are driven by [ros2_control](https://control.ros.org): a C++ loop in the controller manager reads every servo, runs the controllers and writes every target, on the core that `setup_pi.sh` reserves, with real-time priority and its memory locked. Nothing in that loop is Python, so it has no garbage collector and no executor in its timing.

The repository holds four ROS packages:

| Package | Folder | Contents |
|---|---|---|
| `h3215_description` | `h3215/hardware/h3215_description/` | The robot model: `mjcf/h3215.xml`, the URDF **generated from it** (`urdf/h3215.urdf.xacro`, with `config/joints.yaml`), `meshes/` from the CAD (`printed_parts/`, `components/`), and `config/calibration.yaml`, written by hand |
| `h3215_hardware` | `h3215/software/control/h3215_hardware/` | C++: the ros2_control system for the servo bus (`h3215_hardware/StsSystem`), the `servo_tool` command for setting up and checking servos, and their tests |
| `h3215_core` | `h3215/software/core/h3215_core/` | Python: the high-level nodes, `posture` and `monitor` |
| `h3215_bringup` | `h3215/software/core/h3215_bringup/` | The launch files (`control.launch.py`, `camera.launch.py`, `robot.launch.py`) and their configuration (`controllers.yaml`, `core.yaml`) |

## The workspace

The workspace is outside the repository, as ROS expects: `~/h3215_ws`, with the repository linked into its `src/` folder. colcon searches `src/` and finds the four packages wherever they sit (it skips `h3215/software/training/`, which has a `COLCON_IGNORE`); everything it writes (`build/`, `install/`, `log/`) stays in the workspace.

```
~/h3215_ws/
  src/H-3215 -> the repository
  build/  install/  log/
```

`scripts/setup_ros.sh` creates it (set `H3215_WS` for another place) and builds it. After pulling changes, rebuild with `bash scripts/setup_ros.sh --build`, or `colcon build --symlink-install` from the workspace. With `--symlink-install`, launch files and configuration are linked rather than copied, so editing them in the repository needs no rebuild.

## The description comes from the model

`h3215/hardware/h3215_description/mjcf/h3215.xml` is the only description of the robot. The code finds its structure from the names in it:

| Name | What it is |
|---|---|
| `base` | The body with the free joint, holding site `imu` |
| `leg_<i>_link_<j>` | Body `j` of leg `i`, counted from the base. `i` and `j` start at 1, without gaps |
| `leg_<i>_joint_<j>` | The hinge of body `leg_<i>_link_<j>` |
| `leg_<i>_foot` | Site at the foot tip, in the leg's last body |
| `leg_<i>_foot_collision` | Geom of the foot |
| `*_collision` | Geoms that collide in simulation |
| `camera_link` | Site where the depth camera is mounted (optional): the camera driver's frames hang from it |
| keyframe `home` | The default pose. Only its joint angles count: the base height is computed so that the feet touch the ground |

`h3215_train.morphology` (in `h3215/software/training/`) reads them and checks them, with an error naming what's wrong. Training reads the model directly. ROS needs a URDF, and the Pi has no MuJoCo to make one, so on a PC a generator writes it next to the model, in the same package, with the joint lists and the home pose that the leg controller and the posture node need:

```bash
uv run --project h3215/software/training python -m h3215_train.description           # after every model change
uv run --project h3215/software/training python -m h3215_train.description --check   # are they up to date?
```

Commit the generated files with the model. `tests/test_morphology.py` fails if they're stale.

Add a leg, or a joint to every leg, by following the names: training, the description, the servo IDs and the controllers all adapt.

## Servo IDs and calibration

Servo IDs follow the joint order: leg 1 joint 1 is ID 1, leg 1 joint 2 is ID 2, and so on, leg by leg. On H-3215 that gives leg 6 joint 3 ID 18; joints outside the legs, like a neck, come after. Give each servo its ID with `servo_tool` (see [Servos](servos.md)).

Each joint's angle comes from its servo's position:

$$\text{angle} = \text{direction} \cdot (\text{steps} - 2048) \cdot \frac{2\pi}{4096} + \text{offset}$$

`offset` (rad) and `direction` (1 or −1) are in `h3215_description/config/calibration.yaml`, by joint. Joints left out use 0 and 1. That file is written by hand and never generated, so regenerating the description keeps it. Pass another file with `calibration_file:=<path>`.

## Running

On the Pi, after [setting it up](raspberry-pi.md):

```bash
source ~/h3215_ws/install/setup.bash
ros2 launch h3215_bringup robot.launch.py                          # the whole robot
ros2 launch h3215_bringup robot.launch.py use_mock_hardware:=true  # no hardware, no camera
```

| Launch file | Starts | Arguments |
|---|---|---|
| `robot.launch.py` | The whole robot: `control.launch.py`, `camera.launch.py` and the `h3215_core` nodes | `use_mock_hardware`, `port`, `calibration_file`, `camera` (default: on unless the hardware is mocked) |
| `control.launch.py` | The control loop alone: the description, ros2_control, the controllers | `use_mock_hardware`, `port`, `calibration_file` |
| `camera.launch.py` | The RealSense D435i: depth and color at 424 × 240, 30 Hz, and its IMU, under `/camera/camera/` | |

Then:

```bash
ros2 service call /posture/stand std_srvs/srv/Trigger   # torque on, then a smooth move to the home pose
ros2 service call /posture/relax std_srvs/srv/Trigger   # torque off: the robot sits down on its body
ros2 topic echo /battery
```

### The control loop

`control.launch.py` starts `robot_state_publisher` (TF for every link, foot and the camera mount), the controller manager, and two controllers:

| Controller | Type | Does |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | Publishes `/joint_states` (position, velocity, effort) and `/dynamic_joint_states` (also voltage, temperature, error flags) |
| `leg_position_controller` | `position_controllers/JointGroupPositionController` | Takes every leg joint's target on `/leg_position_controller/commands` |

To move every joint at once (one value per leg joint, in the order of `config/joints.yaml`):

```bash
ros2 topic pub --once /leg_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0, 0.35, -0.25, 0, 0.35, -0.25, 0, 0.35, -0.25, 0, 0.35, -0.25, 0, 0.35, -0.25, 0, 0.35, -0.25]}"
```

The policy will replace `leg_position_controller` with a controller of its own, in the same loop.

### What the hardware does

| Step | Servos |
|---|---|
| Configure | Opens the port, pings every ID and names any that don't answer |
| Activate | Sets each goal to the servo's present position, then turns torque on: nothing moves until a command arrives |
| Every cycle | One sync read (position, speed, load of every servo), one sync write of every goal |
| Deactivate | Torque off |

It reports position, velocity and effort for every joint. The effort is the servo's load reading (a PWM duty cycle) times the stall torque: an estimate, not a measured torque. It also reports each servo's supply voltage, temperature and error flags, as the state interfaces `voltage`, `temperature` and `status`.

A cycle with a missing reply keeps the last state. After 5 in a row (`max_missed_cycles`) the hardware reports an error and the controller manager stops it.

Stopping the launch file doesn't deactivate the hardware, so the servos keep holding their last position. To turn torque off first, call `/posture/relax`.

### The high-level nodes

| Node | Does | Parameters (`config/core.yaml`) |
|---|---|---|
| `posture` | `~/stand`: torque on if it was off, then a minimum-jerk move from the present pose to the home pose. `~/relax`: torque off. Before turning torque on it sends the present pose to the leg controller, so the servos don't jump to an old command | `duration` (3 s), `rate` (50 Hz), `stand_on_start` (false). Its joints and home pose come from the model, in `h3215_description/config/joints.yaml` |
| `monitor` | Reads every servo's voltage, temperature and error flags from `/dynamic_joint_states`. Publishes `/battery` (`sensor_msgs/BatteryState`: the median servo voltage and an estimate of the charge from a Li-ion curve) and `/diagnostics` (one status for the battery, one per servo), and logs a warning when one is not OK | `cells` (3), `cell_warn` and `cell_error` (3.4 and 3.2 V per cell), `temperature_warn` and `temperature_error` (60 and 70 °C) |

The charge estimate uses the cells' resting voltage: while the robot walks the voltage sags, so the charge reads low.

### Real time

`config/controllers.yaml` runs the loop at 100 Hz on core 3, with `SCHED_FIFO` priority 90 and locked memory. `setup_pi.sh` allows both for your user. Without them, as in Docker, the controller manager warns and runs with normal scheduling.

## Building on a PC

ROS 2 Jazzy needs Ubuntu 24.04. On any other system, build and test in Docker. The image has the same layout as the Pi: a workspace in `/ws` with the repository, mounted at `/h3215`, linked into `/ws/src`:

```bash
docker build -t h3215-ros h3215/software/control
docker run --rm -it --user $(id -u):$(id -g) -v "$PWD:/h3215" h3215-ros
# inside, in /ws:
colcon build --symlink-install && colcon test --event-handlers console_direct+
source install/setup.bash
ros2 launch h3215_bringup robot.launch.py use_mock_hardware:=true
```

The workspace lives in the container, so its build is gone when you exit.

`colcon test` runs the servo protocol and bus tests against simulated servos on a pseudo-terminal, and the tests of `h3215_core`. The image also has the RealSense driver, so `robot.launch.py` starts there; without a camera, start it with `camera:=false`.
