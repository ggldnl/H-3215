# Hardware

## Models

- [Kinematic model](kinematics.md) (stub)
- [Dynamic model](dynamics.md) (stub)
- [Contact estimation](contact-estimation.md) (stub)

## Components

Chosen so far:

| Part | Choice | Why |
|---|---|---|
| Servos | 19 × Feetech STS3215, 12 V version (C018 or C047): 18 in the legs, 1 in the neck | 30 kg·cm (2.94 N·m) stall torque; report position, speed, load, voltage and temperature over a single bus |
| Servo adapter | 1 or 2 × Waveshare Bus Servo Driver Board, jumpers on **B** (USB) | Connects the Pi to the 1 Mbps half-duplex servo bus over USB |
| Computer | Raspberry Pi 5 | Runs the policy, the servo bus and perception on its CPU; there is no GPU |
| Depth camera | Intel RealSense D435i | Metric depth from about 20 cm, global shutter, built-in IMU; powered over USB 3 |
| Battery | 3S2P Li-ion pack, 18650 cells (Ampace JP30 or equivalent) in a single layer, with a 3S BMS; servos on the battery, a 5 V UBEC for the Pi | Flat (about 20 mm), 65 Wh, about 40 minutes of walking by a conservative estimate; see [Power budget and battery](power.md) |

Still open:

| Part | Requirement | Depends on |
|---|---|---|
| IMU | Mounted rigidly near the center of mass, at 200 Hz or more | Whether the D435i's IMU, mounted with the camera, is close enough to the center of mass |

!!! note "Power"
    How much the servos, the Pi and the camera draw, how the battery is sized and built, and how the servos are protected against overcurrent: [Power budget and battery](power.md).

## Folders

| Folder | Contents |
|---|---|
| `h3215/hardware/h3215_description/` | The robot model, as a ROS package: `mjcf/` (the MJCF model, used by training), `urdf/` (generated from the MJCF for ROS), `config/` (servo calibration, controller joints) |
| `h3215/hardware/h3215_description/meshes/printed_parts/` | The parts you print: STEP to edit, STL to slice |
| `h3215/hardware/h3215_description/meshes/components/` | The parts you buy (servos, Raspberry Pi, servo board, battery, camera): their models, for the assembly and the robot model |

The meshes and the model live together because they change together: when a part changes, update its mesh and the model in the same commit, then regenerate the URDF (`uv run --project h3215/software/training python -m h3215_train.description`).

## Robot model

Until the CAD exists, `h3215/hardware/h3215_description/mjcf/h3215.xml` is a provisional model built from primitive shapes:

![The provisional robot model in its default pose](../media/images/model_provisional.png){ width="560" }

| Property | Value |
|---|---|
| Leg segments | Coxa 68 mm, femur 77 mm, tibia 122 mm (H-0996's proportions) |
| Mass | 2.14 kg: 1.03 kg base (six coxa servos, Pi, servo board, battery, camera) and 185 g per leg |
| Default pose | Femur raised 0.35 rad, tibia −0.25 rad: base at 11.7 cm, feet 22–25 cm from the center |

Frames and numbering:

- **Base frame:** x forward, y left, z up.
- **Legs:** numbered like H-0996, clockwise from above: 1 front-right, 2 middle-right, 3 rear-right, 4 rear-left, 5 middle-left, 6 front-left.
- **Joints:** `leg_<i>_joint_1` to `leg_<i>_joint_3`, from the base outward: coxa, femur, tibia. In each leg frame x points out of the body. Joint 1 turns the leg around z, a positive joint 2 angle lifts the femur, a positive joint 3 angle swings the foot outward.
- **Default pose:** the keyframe `home`. The base height (11.7 cm) is computed from it, with the feet on the ground.

Training and the ROS description find the legs, joints and feet from their names, following the convention in [ROS 2](../software/ros.md#the-description-comes-from-the-model). Keep it when the model is replaced by the CAD export, and regenerate the description after every change.
