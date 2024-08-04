# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import math

from omni.isaac.lab.utils import configclass

from omni.isaac.lab_assets import ISAACLAB_ASSETS_DATA_DIR

# import omni.isaac.lab_tasks.manager_based.manipulation.reach.mdp as mdp
# from omni.isaac.lab_tasks.manager_based.manipulation.reach.reach_env_cfg import ReachEnvCfg
import fetch_project.tasks.manipulation.reach.mdp as mdp
from fetch_project.tasks.manipulation.reach.reach_env_cfg import ReachEnvCfg

# ##
# # Pre-defined configs
# ##
from fetch_project.robots.fetch import FETCH_CFG  # isort: skip

##
# Environment configuration
##

@configclass
class FetchReachEnvCfg(ReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # switch robot to ur10
        self.scene.robot = FETCH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        # override events
        self.events.reset_robot_joints.params["position_range"] = (-0.5, 0.5)
        # override rewards
        self.rewards.end_effector_position_tracking.params["asset_cfg"].body_names = ["wrist_roll_link"]
        self.rewards.end_effector_position_tracking_fine_grained.params["asset_cfg"].body_names = ["wrist_roll_link"]
        self.rewards.end_effector_orientation_tracking.params["asset_cfg"].body_names = ["wrist_roll_link"]
        # override actions
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True
        )
        # override command generator body
        # end-effector is along x-direction
        self.commands.ee_pose.body_name = "wrist_roll_link"
        self.commands.ee_pose.ranges.pitch = (math.pi / 2, math.pi / 2)


@configclass
class FetchReachEnvCfg_PLAY(FetchReachEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
