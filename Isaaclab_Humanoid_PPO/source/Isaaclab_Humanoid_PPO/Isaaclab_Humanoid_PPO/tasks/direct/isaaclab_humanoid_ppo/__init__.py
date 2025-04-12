# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##


gym.register(
    id="Isaaclab-Humanoid-Ppo-v0",
    entry_point=f"{__name__}.isaaclab_humanoid_ppo_env:IsaaclabHumanoidPpoEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaaclab_humanoid_ppo_env_cfg:IsaaclabHumanoidPpoEnvCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)