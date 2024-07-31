# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.utils import configclass

from omni.isaac.lab_assets import ISAACLAB_ASSETS_DATA_DIR

# from omni.isaac.lab_tasks.manager_based.manipulation.cabinet import mdp

# from omni.isaac.lab_tasks.manager_based.manipulation.cabinet.cabinet_env_cfg import (  # isort: skip
#     FRAME_MARKER_SMALL_CFG,
#     CabinetEnvCfg,
# )

import fetch_project.tasks.manipulation.cabinet.mdp as mdp
from fetch_project.tasks.manipulation.cabinet.cabinet_env_cfg import (CabinetEnvCfg, FRAME_MARKER_SMALL_CFG)


##
# Pre-defined configs
##
# from omni.isaac.lab_assets.franka import FRANKA_PANDA_CFG  # isort: skip

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
            effort_limit=1e6,
            velocity_limit=1e6,
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

@configclass
class FetchCabinetEnvCfg(CabinetEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set franka as robot
        self.scene.robot = FETCH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set Actions for the specific robot type (franka)
        self.actions.body_joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"],
            scale=1.0,
            use_default_offset=True,
        )
        self.actions.finger_joint_pos = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["r_gripper_finger_joint", "l_gripper_finger_joint"],
            open_command_expr={"r_gripper_finger_joint": 0.04, "l_gripper_finger_joint": 0.04},
            close_command_expr={"r_gripper_finger_joint": 0.0, "l_gripper_finger_joint": 0.0},
        )

        # Listens to the required transforms
        # IMPORTANT: The order of the frames in the list is important. The first frame is the tool center point (TCP)
        # the other frames are the fingers
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=True,
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/EndEffectorFrameTransformer"),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/wrist_roll_link",
                    name="ee_tcp",
                    offset=OffsetCfg(
                        pos=(0.183, 0.0, 0.0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/l_gripper_finger_link",
                    name="tool_leftfinger",
                    offset=OffsetCfg(
                        pos=(0.015, 0.02, 0.0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/r_gripper_finger_link",
                    name="tool_rightfinger",
                    offset=OffsetCfg(
                        pos=(0.015, -0.02, 0.0),
                    ),
                ),
            ],
        )

        # override rewards
        self.rewards.approach_gripper_handle.params["offset"] = 0.04
        self.rewards.grasp_handle.params["open_joint_pos"] = 0.04
        self.rewards.grasp_handle.params["asset_cfg"].joint_names = ["r_gripper_finger_joint", "l_gripper_finger_joint"]


@configclass
class FetchCabinetEnvCfg_PLAY(FetchCabinetEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
