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
# from omni.isaac.lab_assets import UR10_CFG  # isort: skip

## Import your custom robot configuration here.
import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

FETCH_USD_PATH = f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/FetchRobot/Fetch/fetch.usd"

FETCH_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=FETCH_USD_PATH,
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # base
            "r_wheel_joint": 0.0,
            "l_wheel_joint": 0.0,
            "head_pan_joint": 0.0,
            "head_tilt_joint": 0.0,
            # fetch arm
            "torso_lift_joint": 0.2,
            "shoulder_pan_joint": -1.28,
            "shoulder_lift_joint": 1.51,
            "upperarm_roll_joint": 0.35,
            "elbow_flex_joint": 1.81,
            "forearm_roll_joint": 0.0,
            "wrist_flex_joint": 1.47,
            "wrist_roll_joint": 0.0,
            # # tool
            "r_gripper_finger_joint": 0.01,
            "l_gripper_finger_joint": 0.01,

        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "fetch_torso": ImplicitActuatorCfg(
            joint_names_expr=["torso_lift_joint"],
            effort_limit=1000.0,
            velocity_limit=1.0,
            stiffness=1e5,
            damping=1e3,
        ),
        "fetch_arm": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"],
            effort_limit=87.0,
            velocity_limit=100.0,
            stiffness=1000.0,
            damping=50.0,
        ),
        "fetch_hand": ImplicitActuatorCfg(
            joint_names_expr=["r_gripper_finger_joint", "l_gripper_finger_joint"],
            effort_limit=100.0,
            velocity_limit=0.2,
            stiffness=500,
            damping=50,
        ),
        "fetch_head": ImplicitActuatorCfg(
            joint_names_expr=["head_pan_joint", "head_tilt_joint"],
            effort_limit=10.0,
            velocity_limit=0.2,
            stiffness=100,
            damping=40,
        ),
    },
)

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
        self.events.reset_robot_joints.params["position_range"] = (-0.1, 0.1)
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
