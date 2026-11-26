# Hardware

| Folder | Contents |
|---|---|
| `h3215_description/` | The robot model, as a ROS package: `mjcf/` (used by training), `urdf/` (generated for ROS), `config/` (servo calibration, controller joints) |
| `h3215_description/meshes/printed_parts/` | The parts you print: STEP to edit, STL to slice |
| `h3215_description/meshes/components/` | The parts you buy (servos, Raspberry Pi, servo board, battery, camera): their models, for the assembly and the robot model |

The meshes and the model live together because they change together: when a part changes, update its mesh and the model in the same commit, and regenerate the URDF (`uv run --project h3215/software/training python -m h3215_train.description`, from the repository folder). The current `h3215_description/mjcf/h3215.xml` is provisional (primitive shapes, H-0996's proportions) until the CAD exists.

The parts list, print settings and assembly guide are in the [documentation](../../docs/pages/hardware/index.md).
