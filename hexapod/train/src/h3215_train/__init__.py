"""H-3215 tasks for mjlab.

mjlab imports this package through the "mjlab.tasks" entry point in pyproject.toml, which
registers the tasks below with its train and play commands.
"""

from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import h3215_flat_env_cfg, h3215_rough_env_cfg
from .rl_cfg import h3215_ppo_runner_cfg

register_mjlab_task(
  task_id="Mjlab-Velocity-Rough-H3215",
  env_cfg=h3215_rough_env_cfg(),
  play_env_cfg=h3215_rough_env_cfg(play=True),
  rl_cfg=h3215_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)

register_mjlab_task(
  task_id="Mjlab-Velocity-Flat-H3215",
  env_cfg=h3215_flat_env_cfg(),
  play_env_cfg=h3215_flat_env_cfg(play=True),
  rl_cfg=h3215_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
