"""H-3215 velocity environment configurations.

Both start from mjlab's velocity task (make_velocity_env_cfg), which is sized for robots like
Go1. H-3215 is a 2 kg hexapod whose servos top out at 4.7 rad/s, so commands, terrain, foot
clearance and pushes are scaled down, and the policy gets no linear velocity input: the real
robot has no sensor for it.
"""

import math
from dataclasses import replace

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers import TerminationTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.sensor import (
  ContactMatch,
  ContactSensorCfg,
  GridPatternCfg,
  ObjRef,
  RayCastSensorCfg,
  RingPatternCfg,
  TerrainHeightSensorCfg,
)
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg
from mjlab.terrains.config import (
  flat,
  hf_pyramid_slope,
  hf_pyramid_slope_inv,
  pyramid_stairs,
  pyramid_stairs_inv,
  random_rough,
  wave_terrain,
)
from mjlab.terrains.terrain_generator import TerrainGeneratorCfg

from .h3215.h3215_constants import (
  BASE_BODY,
  FOOT_GEOMS,
  FOOT_SITES,
  H3215_ACTION_SCALE,
  LEG_GEOMS,
  get_h3215_robot_cfg,
)

# mjlab's rough terrain mix at H-3215 scale: steps up to 4 cm instead of 10, gentler slopes
# and smaller tiles. The base clears the ground by about 9 cm in the default pose.
H3215_ROUGH_TERRAINS_CFG = TerrainGeneratorCfg(
  size=(4.0, 4.0),
  border_width=10.0,
  num_rows=10,
  num_cols=20,
  sub_terrains={
    "flat": flat(proportion=0.2),
    "pyramid_stairs": pyramid_stairs(
      proportion=0.2,
      step_height_range=(0.0, 0.04),
      step_width=0.15,
      platform_width=1.0,
      border_width=0.5,
    ),
    "pyramid_stairs_inv": pyramid_stairs_inv(
      proportion=0.2,
      step_height_range=(0.0, 0.04),
      step_width=0.15,
      platform_width=1.0,
      border_width=0.5,
    ),
    "hf_pyramid_slope": hf_pyramid_slope(
      proportion=0.1, slope_range=(0.0, 0.4), platform_width=1.0
    ),
    "hf_pyramid_slope_inv": hf_pyramid_slope_inv(
      proportion=0.1, slope_range=(0.0, 0.4), platform_width=1.0
    ),
    "random_rough": random_rough(
      proportion=0.1, noise_range=(0.005, 0.03), noise_step=0.005
    ),
    "wave_terrain": wave_terrain(proportion=0.1, amplitude_range=(0.0, 0.05)),
  },
  add_lights=True,
)

# Commands H-3215 can follow (m/s, rad/s), and the matching tracking tolerances.
LIN_VEL_X = (-0.3, 0.3)
LIN_VEL_Y = (-0.2, 0.2)
ANG_VEL_Z = (-1.0, 1.0)
LIN_VEL_STD = math.sqrt(0.02)  # mjlab default sqrt(0.25), for commands up to 1-3 m/s
ANG_VEL_STD = math.sqrt(0.2)

FOOT_CLEARANCE = 0.03  # m, swing height target (mjlab default 0.1)


def h3215_rough_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create H-3215 rough terrain velocity configuration."""
  cfg = make_velocity_env_cfg()

  cfg.sim.mujoco.ccd_iterations = 500
  cfg.sim.mujoco.impratio = 10
  cfg.sim.mujoco.cone = "elliptic"
  cfg.sim.contact_sensor_maxmatch = 500

  cfg.scene.entities = {"robot": get_h3215_robot_cfg()}

  assert cfg.scene.terrain is not None
  # A copy: play mode edits the generator, which must not leak into the training config.
  cfg.scene.terrain.terrain_generator = replace(
    H3215_ROUGH_TERRAINS_CFG, curriculum=True
  )

  # Height scan: 0.8 x 0.5 m around the base at 5 cm, about what an elevation map built
  # on the Pi can provide. mjlab's default (1.6 x 1.0 m at 10 cm) is sized for Go1.
  for sensor in cfg.scene.sensors or ():
    if sensor.name == "terrain_scan":
      assert isinstance(sensor, RayCastSensorCfg)
      assert isinstance(sensor.frame, ObjRef)
      sensor.frame.name = BASE_BODY
      sensor.pattern = GridPatternCfg(size=(0.8, 0.5), resolution=0.05)
    if sensor.name == "foot_height_scan":
      assert isinstance(sensor, TerrainHeightSensorCfg)
      sensor.frame = tuple(
        ObjRef(type="site", name=s, entity="robot") for s in FOOT_SITES
      )
      sensor.pattern = RingPatternCfg.single_ring(radius=0.015, num_samples=4)

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=FOOT_GEOMS, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  self_collision_cfg = ContactSensorCfg(
    name="self_collision",
    primary=ContactMatch(mode="subtree", pattern=BASE_BODY, entity="robot"),
    secondary=ContactMatch(mode="subtree", pattern=BASE_BODY, entity="robot"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )
  legs_ground_cfg = ContactSensorCfg(
    name="legs_ground_touch",
    primary=ContactMatch(mode="geom", pattern=LEG_GEOMS, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )
  base_ground_cfg = ContactSensorCfg(
    name="base_ground_touch",
    primary=ContactMatch(mode="geom", pattern=("base_collision",), entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    feet_ground_cfg,
    self_collision_cfg,
    legs_ground_cfg,
    base_ground_cfg,
  )

  # The real robot measures orientation and angular velocity (IMU) but not linear velocity,
  # so only the critic sees it.
  del cfg.observations["actor"].terms["base_lin_vel"]

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = H3215_ACTION_SCALE

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.ranges.lin_vel_x = LIN_VEL_X
  twist_cmd.ranges.lin_vel_y = LIN_VEL_Y
  twist_cmd.ranges.ang_vel_z = ANG_VEL_Z
  # The default curriculum widens commands toward 3 m/s, far beyond these servos.
  cfg.curriculum.pop("command_vel", None)

  cfg.viewer.body_name = BASE_BODY
  cfg.viewer.distance = 0.9
  cfg.viewer.elevation = -15.0

  cfg.events["foot_friction"].params["asset_cfg"].geom_names = FOOT_GEOMS
  cfg.events["base_com"].params["asset_cfg"].body_names = (BASE_BODY,)
  cfg.events["base_com"].params["ranges"] = {
    0: (-0.015, 0.015),
    1: (-0.015, 0.015),
    2: (-0.01, 0.01),
  }
  cfg.events["push_robot"].params["velocity_range"] = {
    "x": (-0.2, 0.2),
    "y": (-0.2, 0.2),
    "z": (-0.1, 0.1),
    "roll": (-0.3, 0.3),
    "pitch": (-0.3, 0.3),
    "yaw": (-0.4, 0.4),
  }

  cfg.rewards["track_linear_velocity"].params["std"] = LIN_VEL_STD
  cfg.rewards["track_angular_velocity"].params["std"] = ANG_VEL_STD

  cfg.rewards["pose"].params["std_standing"] = {
    r".*_(coxa|femur)_joint": 0.05,
    r".*_tibia_joint": 0.1,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    r".*_(coxa|femur)_joint": 0.3,
    r".*_tibia_joint": 0.6,
  }
  cfg.rewards["pose"].params["std_running"] = {
    r".*_(coxa|femur)_joint": 0.3,
    r".*_tibia_joint": 0.6,
  }

  cfg.rewards["upright"].params["asset_cfg"].body_names = (BASE_BODY,)
  cfg.rewards["upright"].params["terrain_sensor_names"] = ("terrain_scan",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = (BASE_BODY,)

  for reward_name in ["foot_clearance", "foot_slip"]:
    cfg.rewards[reward_name].params["asset_cfg"].site_names = FOOT_SITES
  cfg.rewards["foot_clearance"].params["target_height"] = FOOT_CLEARANCE
  cfg.rewards["foot_swing_height"].params["target_height"] = FOOT_CLEARANCE

  cfg.rewards["body_ang_vel"].weight = 0.0
  cfg.rewards["angular_momentum"].weight = 0.0
  cfg.rewards["air_time"].weight = 0.0

  cfg.rewards["self_collisions"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-0.1,
    params={"sensor_name": self_collision_cfg.name},
  )
  cfg.rewards["leg_collision"] = RewardTermCfg(
    func=mdp.self_collision_cost,
    weight=-0.1,
    params={"sensor_name": legs_ground_cfg.name},
  )

  # Rough ground tilts the body a lot, so orientation alone doesn't end an episode: the
  # base touching the ground does.
  cfg.terminations.pop("fell_over", None)
  cfg.terminations["illegal_contact"] = TerminationTermCfg(
    func=mdp.illegal_contact,
    params={"sensor_name": base_ground_cfg.name},
  )

  if play:
    # Effectively infinite episode length.
    cfg.episode_length_s = int(1e9)

    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.terminations.pop("out_of_terrain_bounds", None)
    cfg.curriculum = {}
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

    if cfg.scene.terrain.terrain_generator is not None:
      cfg.scene.terrain.terrain_generator.curriculum = False
      cfg.scene.terrain.terrain_generator.num_cols = 5
      cfg.scene.terrain.terrain_generator.num_rows = 5
      cfg.scene.terrain.terrain_generator.border_width = 5.0

  return cfg


def h3215_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create H-3215 flat terrain velocity configuration."""
  cfg = h3215_rough_env_cfg(play=play)

  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  # No terrain to scan and nothing to bump into on a plane.
  remove_sensors = {"terrain_scan", "self_collision", "legs_ground_touch"}
  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name not in remove_sensors
  )
  del cfg.observations["actor"].terms["height_scan"]
  del cfg.observations["critic"].terms["height_scan"]
  cfg.rewards["upright"].params.pop("terrain_sensor_names", None)
  for key in ("self_collisions", "leg_collision"):
    cfg.rewards.pop(key, None)

  cfg.terminations.pop("out_of_terrain_bounds", None)
  cfg.terminations["fell_over"] = TerminationTermCfg(
    func=mdp.bad_orientation,
    params={"limit_angle": math.radians(70.0)},
  )
  cfg.curriculum.pop("terrain_levels", None)

  return cfg
