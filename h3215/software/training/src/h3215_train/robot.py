"""The robot in mjlab: its model, its STS3215 actuators and its collisions.

The structure (legs, joints, feet, default pose) is read from the model by
h3215_train.morphology, so a model with other legs needs no change here.
"""

import math
import os
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from . import morphology

##
# MJCF.
##

# The model lives in the description package, h3215/hardware/h3215_description/mjcf/, so
# training runs from a clone of the repository (uv installs this package in editable mode).
# H3215_MODEL=<path> trains another model that follows the same naming convention.
H3215_XML = Path(os.environ.get("H3215_MODEL", morphology.DEFAULT_MODEL))
assert H3215_XML.exists(), f"robot model not found at {H3215_XML}"

ROBOT = morphology.load(H3215_XML)


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(H3215_XML))
    # The default pose reaches mjlab through INIT_STATE. A keyframe sized for the robot alone
    # would no longer fit once mjlab adds the terrain and the sensors to the scene.
    for key in list(spec.keys):
        spec.delete(key)
    return spec


##
# Names shared with env_cfgs.py.
##

BASE_BODY = morphology.BASE_BODY
FOOT_SITES = ROBOT.foot_sites
FOOT_GEOMS = ROBOT.foot_geoms
LEG_GEOMS = ROBOT.leg_collision_geoms
JOINTS_REGEX = r"leg_\d+_joint_\d+"
# The last joint of each leg (the tibia on H-3215) moves the most while walking.
LAST_JOINTS = tuple(leg.joints[-1] for leg in ROBOT.legs)
OTHER_LEG_JOINTS = tuple(j for j in ROBOT.leg_joints if j not in LAST_JOINTS)

##
# Actuator config.
##

# Feetech STS3215, 12 V version (C018 / C047): 30 kg*cm stall torque, 0.222 s/60 deg no load.
STALL_TORQUE = 30 * 9.81 / 100  # 2.94 N*m
NO_LOAD_SPEED = math.radians(60) / 0.222  # 4.7 rad/s

# Joint dynamics from the sts3215 class of MuJoCo Menagerie's SO-101 model. The viscous
# damping is what caps the speed: STALL_TORQUE / 0.60 = 4.9 rad/s, close to NO_LOAD_SPEED.
ARMATURE = 0.028  # kg*m^2, rotor inertia reflected through the 1:345 gearbox
VISCOUS_DAMPING = 0.60  # N*m*s/rad
FRICTIONLOSS = 0.052  # N*m

# PD gains from the armature, like mjlab's own robots: 10 Hz natural frequency, damping
# ratio 2. The real servo's position loop is stiffer, which only changes errors below about
# 1.5 deg: beyond that both saturate at the stall torque.
NATURAL_FREQ = 10 * 2.0 * math.pi
DAMPING_RATIO = 2.0
STIFFNESS = ARMATURE * NATURAL_FREQ**2  # 110 N*m/rad
DAMPING = 2 * DAMPING_RATIO * ARMATURE * NATURAL_FREQ  # 7.0 N*m*s/rad

STS3215_ACTUATOR_CFG = BuiltinPositionActuatorCfg(
    target_names_expr=(JOINTS_REGEX,),
    stiffness=STIFFNESS,
    damping=DAMPING,
    effort_limit=STALL_TORQUE,
    armature=ARMATURE,
    frictionloss=FRICTIONLOSS,
    viscous_damping=VISCOUS_DAMPING,
)

# Range the policy's action [-1, 1] spans around the default pose. mjlab's robots use
# 0.25 * effort_limit / stiffness, which would be 0.4 deg here: hobby servos saturate within
# a degree or two, so the range is set directly.
H3215_ACTION_SCALE = 0.5  # rad

##
# Keyframes.
##

# The default pose from the model's "home" keyframe. On H-3215: femur raised 20 deg, tibia
# 6 deg past vertical, the feet 22 cm from the center. The base starts 1 cm above the height
# where the feet touch the ground.
INIT_STATE = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, ROBOT.base_height + 0.01),
    joint_pos=dict(ROBOT.home),
    joint_vel={".*": 0.0},
)

##
# Collision config.
##

_foot_regex = r"^leg_\d+_foot_collision$"
_collision_regex = r".*_collision"

# Printed TPU feet on indoor floors and outdoor ground. Domain randomization varies it.
FOOT_FRICTION = 0.8

# Only the feet collide: the cheapest setup, for quick experiments.
FEET_ONLY_COLLISION = CollisionCfg(
    geom_names_expr=(_foot_regex,),
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(FOOT_FRICTION,),
    solimp=(0.9, 0.95, 0.023),
)

# Everything collides. Feet get friction (condim 3), the rest is frictionless (condim 1).
FULL_COLLISION = CollisionCfg(
    geom_names_expr=(_collision_regex,),
    contype=1,
    conaffinity=1,
    solref=(0.01, 1),
    condim={_foot_regex: 3, _collision_regex: 1},
    priority={_foot_regex: 1, ".*": 0},
    friction={_foot_regex: (FOOT_FRICTION,)},
)

##
# Final config.
##

H3215_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(STS3215_ACTUATOR_CFG,),
    soft_joint_pos_limit_factor=0.9,
)


def get_h3215_robot_cfg() -> EntityCfg:
    """Get a fresh H-3215 robot configuration instance.

    Returns a new EntityCfg each time, so configs shared between tasks are never mutated.
    """
    return EntityCfg(
        init_state=INIT_STATE,
        collisions=(FULL_COLLISION,),
        spec_fn=get_spec,
        articulation=H3215_ARTICULATION,
    )


if __name__ == "__main__":
    import mujoco.viewer as viewer
    from mjlab.entity.entity import Entity

    robot = Entity(get_h3215_robot_cfg())

    viewer.launch(robot.spec.compile())
