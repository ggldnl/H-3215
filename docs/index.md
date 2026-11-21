# H-3215

An open-source hexapod with 18 Feetech STS3215 smart servos and a locomotion policy trained with reinforcement learning, running on a Raspberry Pi 5. It is named after its servos, like its predecessor [H-0996](https://github.com/ggldnl/H-0996) (MG996R servos).

<figure markdown>
  <video src="media/videos/flat_policy_sim.mp4" autoplay loop muted playsinline width="640"></video>
  <figcaption>The first flat-ground policy in simulation, on the provisional robot model: forward, then turning on the spot.</figcaption>
</figure>

The servos are the biggest change between the two robots. H-0996's MG996R can only be sent a target angle and never report back, while the STS3215 are stronger and report their position and load, so H-3215 can sense when a foot touches down, adapt to uneven ground and run a learned gait that reacts to its own joint state.

## Goals

- **Follow velocity commands:** walk forward, backward and sideways and turn at a commanded speed.
- **Map the terrain around it** (small obstacles, uneven ground), keep the map up to date while walking, and use it to place its feet and choose where to go.
- **Be reproducible:** every part, setting and experiment documented here, so anyone can build the robot and train the same policy.

## Status

| Area | State |
|---|---|
| Servo driver and bus timing | Working, tested with 3 servos |
| Raspberry Pi real-time setup | Working |
| Robot model | Provisional: primitive shapes until the CAD exists |
| Flat-ground policy | Trains in about 10 minutes in simulation |
| Mechanical design | Not started |
| Battery and camera | Chosen: 3S2P Li-ion pack, RealSense D435i |
| IMU | To be chosen |
| ROS 2 integration | Being discussed |

## Where to start

<div class="grid cards" markdown>

-   __Hardware__

    ---

    The parts chosen so far, the ones still open, and the robot model used for training.

    [Hardware](hardware/index.md)

-   __Software__

    ---

    How the code is organized, from the servo driver on the Pi to the training tasks.

    [Software](software/index.md)

-   __Raspberry Pi setup__

    ---

    Configure a Pi for the 50 Hz control loop and install the Python environment.

    [Raspberry Pi setup](software/raspberry-pi.md)

-   __Training in simulation__

    ---

    Train a walking policy with mjlab on any machine with an NVIDIA GPU.

    [Training](software/training.md)

</div>

## Why one repository

H-0996 is split into eight repositories linked by submodules. That kept it modular, but every firmware or hardware change meant updating the pinned commit in each repository that included it. H-3215 keeps the modularity in folders instead:

| Folder | Contents |
|---|---|
| `hardware/` | Printed parts, bought parts, and the robot model, which changes with the printed parts |
| `hexapod/` | The software: the robot package and the mjlab training project (`hexapod/train/`) |
| `docs/` | This documentation, with its images and videos in `docs/media/` |
| `scripts/` | Setup you run once: Pi configuration, Python environment, servo IDs |
| `tests/` | Checks and benchmarks |

One commit can change a printed part, the robot model and the training code together, so they can't drift apart.
