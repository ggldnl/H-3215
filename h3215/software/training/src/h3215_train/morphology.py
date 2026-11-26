"""The robot's structure, found in its MJCF model by naming convention.

The convention is described at the top of h3215/hardware/h3215_description/mjcf/h3215.xml. Legs, their joints,
links, feet and collision geoms are discovered from the names, so a model with another number
of legs, or another number of joints per leg, needs no code change.

    from h3215_train import morphology
    robot = morphology.load()            # h3215/hardware/h3215_description/mjcf/h3215.xml
    robot.joints                         # ('leg_1_joint_1', 'leg_1_joint_2', ...)
    robot.home['leg_1_joint_2']          # 0.35

Needs mujoco, which compiles the model: classes, defaults and orientations are resolved
exactly as in simulation.
"""

import re
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
DESCRIPTION = REPO_ROOT / "h3215" / "hardware" / "h3215_description"  # the ROS package with the model
DEFAULT_MODEL = DESCRIPTION / "mjcf" / "h3215.xml"

BASE_BODY = "base"
IMU_SITE = "imu"
HOME_KEY = "home"
_LEG_JOINT = re.compile(r"leg_(\d+)_joint_(\d+)")


class ModelError(ValueError):
    """The model doesn't follow the naming convention."""


@dataclass(frozen=True)
class Leg:
    index: int
    links: tuple[str, ...]  # bodies from the base outward
    joints: tuple[str, ...]  # one hinge per link, same order
    foot_site: str
    foot_geom: str
    collision_geoms: tuple[str, ...]  # every *_collision geom of the leg except the foot


@dataclass(frozen=True)
class Morphology:
    legs: tuple[Leg, ...]
    other_joints: tuple[str, ...]  # joints outside the legs (a neck), in model order
    home: dict[str, float]  # joint -> default angle in rad, from keyframe "home"
    ranges: dict[str, tuple[float, float]]  # joint -> (lower, upper) in rad
    base_height: float  # base origin above flat ground in the home pose, feet touching it
    imu_site: str | None

    @property
    def leg_joints(self) -> tuple[str, ...]:
        """Leg joints, leg by leg, each from the base outward."""
        return tuple(j for leg in self.legs for j in leg.joints)

    @property
    def joints(self) -> tuple[str, ...]:
        """Every actuated joint: the leg joints, then the others."""
        return self.leg_joints + self.other_joints

    @property
    def foot_sites(self) -> tuple[str, ...]:
        return tuple(leg.foot_site for leg in self.legs)

    @property
    def foot_geoms(self) -> tuple[str, ...]:
        return tuple(leg.foot_geom for leg in self.legs)

    @property
    def leg_collision_geoms(self) -> tuple[str, ...]:
        return tuple(g for leg in self.legs for g in leg.collision_geoms)

    @property
    def servo_ids(self) -> dict[str, int]:
        """Servo bus IDs by convention: 1, 2, ... in the order of `joints`."""
        return {joint: i for i, joint in enumerate(self.joints, start=1)}


def load(path: str | Path = DEFAULT_MODEL) -> Morphology:
    return from_model(mujoco.MjModel.from_xml_path(str(path)))


def from_model(model: mujoco.MjModel) -> Morphology:
    def name(obj, i):
        return mujoco.mj_id2name(model, obj, i) or ""

    def find(obj, n):
        return mujoco.mj_name2id(model, obj, n)

    body, joint, site, geom = (
        mujoco.mjtObj.mjOBJ_BODY,
        mujoco.mjtObj.mjOBJ_JOINT,
        mujoco.mjtObj.mjOBJ_SITE,
        mujoco.mjtObj.mjOBJ_GEOM,
    )

    base = find(body, BASE_BODY)
    if base < 0:
        raise ModelError(f'no body named "{BASE_BODY}"')
    free = [j for j in range(model.njnt) if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
    if [model.jnt_bodyid[j] for j in free] != [base]:
        raise ModelError(f'the model needs exactly one free joint, on body "{BASE_BODY}"')

    # Group the leg joints: {leg: {j: joint id}}
    found: dict[int, dict[int, int]] = {}
    other = []
    for j in range(model.njnt):
        jname = name(joint, j)
        if j in free:
            continue
        match = _LEG_JOINT.fullmatch(jname)
        if match:
            found.setdefault(int(match[1]), {})[int(match[2])] = j
        elif jname.startswith("leg_"):
            raise ModelError(
                f'joint "{jname}" starts like a leg joint but isn\'t leg_<i>_joint_<j>'
            )
        else:
            other.append(j)
    if not found:
        raise ModelError("no leg joints: name them leg_<i>_joint_<j>")
    _check_numbering("legs", sorted(found))

    legs = []
    for i in sorted(found):
        _check_numbering(f"joints of leg {i}", sorted(found[i]))
        links, joints, parent = [], [], base
        for k in sorted(found[i]):
            j = found[i][k]
            jname, lname = name(joint, j), f"leg_{i}_link_{k}"
            b = model.jnt_bodyid[j]
            if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ModelError(f'joint "{jname}" must be a hinge')
            if name(body, b) != lname:
                raise ModelError(
                    f'joint "{jname}" must belong to body "{lname}", not "{name(body, b)}"'
                )
            if model.body_jntnum[b] != 1:
                raise ModelError(f'body "{lname}" must have exactly one joint')
            if model.body_parentid[b] != parent:
                raise ModelError(f'body "{lname}" must be a child of "{name(body, parent)}"')
            links.append(lname)
            joints.append(jname)
            parent = b

        foot_site, foot_geom = f"leg_{i}_foot", f"leg_{i}_foot_collision"
        s, g = find(site, foot_site), find(geom, foot_geom)
        if s < 0 or model.site_bodyid[s] != parent:
            raise ModelError(f'leg {i} needs site "{foot_site}" in its last body "{links[-1]}"')
        if g < 0 or model.geom_bodyid[g] != parent:
            raise ModelError(f'leg {i} needs geom "{foot_geom}" in its last body "{links[-1]}"')
        leg_bodies = {find(body, link) for link in links}
        collision = tuple(
            name(geom, n)
            for n in range(model.ngeom)
            if model.geom_bodyid[n] in leg_bodies
            and name(geom, n).endswith("_collision")
            and name(geom, n) != foot_geom
        )
        legs.append(Leg(i, tuple(links), tuple(joints), foot_site, foot_geom, collision))

    actuated = [found[i][k] for i in sorted(found) for k in sorted(found[i])] + other
    for j in other:
        if model.jnt_type[j] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
            raise ModelError(f'joint "{name(joint, j)}" must be a hinge or a slide')

    key = find(mujoco.mjtObj.mjOBJ_KEY, HOME_KEY)
    if key < 0:
        raise ModelError(f'no keyframe named "{HOME_KEY}" with the default pose')
    qpos = model.key_qpos[key].copy()
    home = {name(joint, j): float(qpos[model.jnt_qposadr[j]]) for j in actuated}
    ranges = {}
    for j in actuated:
        jname = name(joint, j)
        lower, upper = model.jnt_range[j] if model.jnt_limited[j] else (-np.inf, np.inf)
        ranges[jname] = (float(lower), float(upper))
        if not lower <= home[jname] <= upper:
            raise ModelError(f'keyframe "{HOME_KEY}" puts "{jname}" outside its range')

    return Morphology(
        legs=tuple(legs),
        other_joints=tuple(name(joint, j) for j in other),
        home=home,
        ranges=ranges,
        base_height=_standing_height(model, qpos, base, legs),
        imu_site=IMU_SITE if find(site, IMU_SITE) >= 0 else None,
    )


def _check_numbering(what, numbers):
    if numbers != list(range(1, len(numbers) + 1)):
        raise ModelError(f"{what} must be numbered 1, 2, ... without gaps, found {numbers}")


def _standing_height(model, qpos, base, legs):
    """Height of the base origin when the lowest foot touches flat ground, base level."""
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    adr = model.jnt_qposadr[model.body_jntadr[base]]
    data.qpos[adr : adr + 7] = [0, 0, 0, 1, 0, 0, 0]
    mujoco.mj_kinematics(model, data)
    lowest = np.inf
    for leg in legs:
        g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg.foot_geom)
        # Bounding sphere radius: exact for the usual spherical foot
        lowest = min(lowest, data.geom_xpos[g][2] - model.geom_rbound[g])
    return float(-lowest)
