# Hardware

| Folder | Contents |
|---|---|
| `printed/` | Parts you print: STEP files to edit, STL files to slice |
| `components/` | Parts you buy (servos, Raspberry Pi, servo board, battery, camera): their models, for the assembly and the robot model |
| `model/` | The robot model used in simulation and training: MJCF now, URDF later |

`model/` lives here because it changes with the printed parts: when a part changes, update the model in the same commit. The current `model/h3215.xml` is provisional (primitive shapes, H-0996's proportions) until the CAD exists.

The parts list, print settings and assembly guide are in the [documentation](../docs/).
