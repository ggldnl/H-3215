# H-3215

A hexapod robot with 18 Feetech STS3215 smart servos and an RL locomotion policy, running on a Raspberry Pi 5.
It is named after its servos, like its predecessor [H-0996](https://github.com/ggldnl/H-0996) (MG996R servos).

The documentation is in [`docs/`](docs/pages/index.md).

## Raspberry Pi

Ubuntu 24.04 on the Pi, then, from the repository folder:

```bash
sudo bash scripts/setup_pi.sh       # real-time settings, serial port access
sudo reboot
bash scripts/setup_ros.sh           # ROS 2 Jazzy, then builds ~/h3215_ws
source ~/h3215_ws/install/setup.bash
```

After pulling changes, rebuild with `bash scripts/setup_ros.sh --build`.

### Servos

New servos all have ID 1. Connect one at a time and give each its ID (1 to 18, leg by leg):

```bash
ros2 run h3215_hardware servo_tool set-id 1 <id>
```

With all of them connected:

```bash
ros2 run h3215_hardware servo_tool fast-reply      # removes the reply delay
ros2 run h3215_hardware servo_tool check           # status and bus timing
ros2 run h3215_hardware servo_tool check --move    # slow sine on every servo
```

### Running the robot

```bash
ros2 launch h3215_bringup robot.launch.py           # control loop, camera, posture, monitor
ros2 service call /posture/stand std_srvs/srv/Trigger
ros2 service call /posture/relax std_srvs/srv/Trigger
ros2 topic echo /battery
ros2 topic echo /diagnostics
```

Launch arguments: `port:=/dev/ttyACM0`, `camera:=false`, `use_mock_hardware:=true`. `control.launch.py` and `camera.launch.py` start the two parts alone.

## PC

### Training

Needs an NVIDIA GPU and [uv](https://docs.astral.sh/uv/):

```bash
cd h3215/software/training
uv sync
uv run play Mjlab-Velocity-Flat-H3215 --agent zero
uv run train Mjlab-Velocity-Flat-H3215 --env.scene.num-envs 4096
uv run play Mjlab-Velocity-Flat-H3215 --checkpoint-file logs/rsl_rl/h3215_velocity/<run>/model_<n>.pt
```

The rough terrain task is `Mjlab-Velocity-Rough-H3215`.

### After changing the model

The URDF and the joint lists are generated from `h3215/hardware/h3215_description/mjcf/h3215.xml`:

```bash
uv run --project h3215/software/training python -m h3215_train.description           # regenerate
uv run --project h3215/software/training python -m h3215_train.description --check   # is it up to date?
uv run --project h3215/software/training python tests/test_morphology.py
```

### ROS packages, in Docker

```bash
docker build -t h3215-ros h3215/software/control
docker run --rm -it --user $(id -u):$(id -g) -v "$PWD:/h3215" h3215-ros
# inside, in /ws:
colcon build --symlink-install && colcon test --event-handlers console_direct+
source install/setup.bash
ros2 launch h3215_bringup robot.launch.py use_mock_hardware:=true
```

### Documentation

```bash
uvx zensical@0.1.1 serve            # http://localhost:8215
```
