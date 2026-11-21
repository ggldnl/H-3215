# H-3215

A hexapod robot with 18 Feetech STS3215 smart servos and an RL locomotion policy, running on a Raspberry Pi 5.
It is named after its servos, like its predecessor [H-0996](https://github.com/ggldnl/H-0996) (MG996R servos).

The servos are the biggest change between the two robots. H-0996's MG996R can only be sent a target angle and never report back, while the STS3215 are stronger and report their position and load, so H-3215 can sense when a foot touches down, adapt to uneven ground and run a learned gait that reacts to its own joint state.

H-0996 is split into eight repositories linked by submodules. That kept it modular, but every firmware or hardware change meant updating the pinned commit in each repository that included it. H-3215 keeps the modularity in folders instead: one commit can change a printed part, the robot model and the training code together, so they can't drift apart, and `hexapod/train/` is a self-contained mjlab project that the robot code never imports.

## Layout

| Path | Contents |
|---|---|
| `hardware/` | Printed parts (`printed/`), bought parts (`components/`) and the robot model for simulation (`model/h3215.xml`) |
| `hexapod/` | The software: the robot package (`src/hexapod/`, so far the servo driver) and the mjlab training project (`train/`) |
| `docs/` | The documentation, with its images and videos in `docs/media/` |
| `scripts/` | Setup you run once: Pi configuration, Python environment, servo IDs |
| `tests/` | Checks and benchmarks: `test_sts.py` (no hardware), `servo_check.py`, `bench_loop.py` |

## Documentation

Setup guides, the parts list and design notes live in [`docs/`](docs/index.md):

- [Hardware](docs/hardware/index.md): parts chosen so far, the open ones, the robot model
- [Power budget and battery](docs/hardware/power.md): consumption, battery sizing, overcurrent protection
- [Raspberry Pi setup](docs/software/raspberry-pi.md): system configuration and Python environment
- [Servos](docs/software/servos.md): IDs, reply delay, bus timing
- [Training in simulation](docs/software/training.md): mjlab tasks, servo model, first results
- [Tests and benchmarks](docs/software/tests.md)

It builds into a website with [Zensical](https://zensical.org). To preview it while editing, from the repository folder:

```bash
uvx zensical@0.1.1 serve
```

Then open http://localhost:8000. On GitHub, `.github/workflows/docs.yml` publishes it to GitHub Pages on every push to `main`.
