"""H-3215 constants: the robot model, its STS3215 actuators and its collisions."""

import math
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

##
# MJCF.
##

# The model lives with the printed parts it is built from, in the repository's hardware/
# folder, so train/ runs from a clone of the repository (uv installs it in editable mode).
REPO_ROOT = Path(__file__).resolve().parents[5]
H3215_XML: Path = REPO_ROOT / "hardware" / "model" / "h3215.xml"
assert H3215_XML.exists(), f"robot model not found at {H3215_XML}"


def get_spec() -> mujoco.MjSpec:
  return mujoco.MjSpec.from_file(str(H3215_XML))


##
# Names shared with env_cfgs.py.
##

LEGS = (1, 2, 3, 4, 5, 6)
BASE_BODY = "base"
FOOT_SITES = tuple(f"leg_{i}_foot" for i in LEGS)
FOOT_GEOMS = tuple(f"leg_{i}_foot_collision" for i in LEGS)
LEG_GEOMS = tuple(
  f"leg_{i}_{part}_collision" for i in LEGS for part in ("femur", "tibia")
)
JOINTS_REGEX = r"leg_[1-6]_(coxa|femur|tibia)_joint"

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

# Femur raised 20 deg, tibia 6 deg past vertical: the feet land 22 cm from the center and
# the base origin sits at NOMINAL_BASE_HEIGHT.
COXA_DEFAULT = 0.0
FEMUR_DEFAULT = 0.35
TIBIA_DEFAULT = -0.25
NOMINAL_BASE_HEIGHT = 0.117

INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, NOMINAL_BASE_HEIGHT + 0.01),
  joint_pos={
    ".*_coxa_joint": COXA_DEFAULT,
    ".*_femur_joint": FEMUR_DEFAULT,
    ".*_tibia_joint": TIBIA_DEFAULT,
  },
  joint_vel={".*": 0.0},
)

##
# Collision config.
##

_foot_regex = r"^leg_[1-6]_foot_collision$"
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
