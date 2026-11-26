# Training in simulation

The training code is `h3215_train.tasks`, in `h3215/software/training/`, set up like mjlab's [ANYmal C example](https://github.com/mujocolab/anymal_c_velocity): mjlab is an ordinary dependency, and an entry point in `h3215/software/training/pyproject.toml` adds the H-3215 tasks to mjlab's own `train` and `play` commands. Nothing is forked, so updating mjlab means changing its version in the lock file.

It needs an NVIDIA GPU and [uv](https://docs.astral.sh/uv/), which installs the exact versions in `h3215/software/training/uv.lock`:

```bash
cd h3215/software/training
uv sync                                                   # once, and after every lock file change
uv run play Mjlab-Velocity-Flat-H3215 --agent zero        # sanity check: the robot holds its default pose
uv run train Mjlab-Velocity-Flat-H3215 --env.scene.num-envs 4096
uv run play Mjlab-Velocity-Flat-H3215 --checkpoint-file logs/rsl_rl/h3215_velocity/<run>/model_<n>.pt
```

Runs log to Weights & Biases; add `--agent.logger tensorboard` to keep them local. Logs and checkpoints go to `h3215/software/training/logs/`, which git ignores.

!!! note "mjviser is held below 0.0.17"
    mjlab 1.6 accepts any mjviser from 0.0.14, but 0.0.17 needs MuJoCo 4 (`mujoco-core`), which installs over the MuJoCo 3.11 that mjlab needs and breaks it. `h3215/software/training/pyproject.toml` constrains it until mjlab moves to MuJoCo 4.

## Tasks

| Task | Terrain | Policy inputs |
|---|---|---|
| `Mjlab-Velocity-Flat-H3215` | Plane | IMU, joint states, last action, command |
| `Mjlab-Velocity-Rough-H3215` | Steps up to 4 cm, slopes, noise | The same, plus a 0.8 × 0.5 m height scan around the base |

Both start from mjlab's velocity task, which is sized for robots like Unitree Go1, and scale it to H-3215:

| Setting | mjlab default | H-3215 | Why |
|---|---|---|---|
| Commands | ±1 m/s, curriculum up to 3 m/s | ±0.3 m/s forward, ±0.2 m/s sideways, ±1 rad/s turning | The servos top out at 4.7 rad/s |
| Swing height target | 10 cm | 3 cm | Legs a third of Go1's |
| Pushes | ±0.5 m/s | ±0.2 m/s | A 2 kg robot |
| Linear velocity input | Policy and critic | Critic only | The real robot has no sensor for it |

## The robot model

The tasks read the robot's structure from `h3215/hardware/h3215_description/mjcf/h3215.xml` through `h3215_train.morphology`: the legs, their joints, feet and collision geoms, and the default pose from the keyframe `home`. Nothing in `h3215_train` names a joint, so a model with other legs trains without code changes as long as it follows the [naming convention](ros.md#the-description-comes-from-the-model). To train another model:

```bash
H3215_MODEL=/path/to/model.xml uv run train Mjlab-Velocity-Flat-H3215 --env.scene.num-envs 4096
```

Per-joint settings are written by role: the pose reward's tolerances are twice as wide for the last joint of each leg (the tibia) as for the others.

## Servo model

The robot model is `h3215/hardware/h3215_description/mjcf/h3215.xml`. The servos are added by `h3215/software/training/src/h3215_train/robot.py`:

| Parameter | Value | Source |
|---|---|---|
| Torque limit | 2.94 N·m | Datasheet: 30 kg·cm stall torque at 12 V |
| Armature | 0.028 kg·m² | Rotor inertia through the 1:345 gearbox, from MuJoCo Menagerie's [SO-101 model](https://github.com/google-deepmind/mujoco_menagerie/tree/main/robotstudio_so101) |
| Joint damping | 0.60 N·m·s/rad | Menagerie; caps the speed at 4.9 rad/s, close to the datasheet's 4.7 |
| Friction loss | 0.052 N·m | Menagerie |
| Stiffness, damping | 110 N·m/rad, 7.0 N·m·s/rad | 10 Hz natural frequency on the armature, damping ratio 2, as for mjlab's own robots |
| Action scale | ±0.5 rad around the default pose | Set directly: hobby servos saturate within a degree or two, so mjlab's formula would give ±0.4° |

## First results

A flat-ground run on the provisional model: 600 iterations with 4096 robots, about 10 minutes on an RTX 5070 Ti. Holding each command for 6 seconds, averaged over 16 robots:

| Command | Reached |
|---|---|
| Forward 0.2 / 0.3 m/s | 0.205 / 0.296 m/s |
| Backward 0.2 m/s | 0.210 m/s |
| Left 0.15 m/s | 0.153 m/s |
| Turn 0.8 rad/s | 0.73 rad/s |

No falls, and the body stays within 0.5° of level.

## The exported policy

Every checkpoint is also exported to ONNX, next to it in the run folder. Its metadata holds the joint order, PD gains, default pose, observation names, scales and history lengths, and the action scale: that file is all the robot needs to run the policy.

## Using a local mjlab checkout

To train with a local mjlab instead, such as a fork, install this package into that checkout's environment, then run its `train` and `play` from `h3215/software/training/`:

```bash
uv pip install --python <mjlab>/.venv --no-deps -e h3215/software/training
```
