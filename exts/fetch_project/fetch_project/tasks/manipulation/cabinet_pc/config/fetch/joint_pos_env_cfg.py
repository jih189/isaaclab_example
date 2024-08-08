# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from omni.isaac.lab.sensors import FrameTransformerCfg
from omni.isaac.lab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from omni.isaac.lab.utils import configclass

# from omni.isaac.lab_tasks.manager_based.manipulation.cabinet import mdp

# from omni.isaac.lab_tasks.manager_based.manipulation.cabinet.cabinet_env_cfg import (  # isort: skip
#     FRAME_MARKER_SMALL_CFG,
#     CabinetEnvCfg,
# )

import fetch_project.tasks.manipulation.cabinet_pc.mdp as mdp
from fetch_project.tasks.manipulation.cabinet_pc.cabinet_env_cfg import (CabinetEnvCfg, FRAME_MARKER_SMALL_CFG)

# ##
# # Pre-defined configs
# ##
from fetch_project.robots.fetch import FETCH_CFG  # isort: skip

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.sensors.camera import TiledCameraCfg, CameraCfg


@configclass
class FetchCabinetEnvCfg(CabinetEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set fetch as robot
        self.scene.robot = FETCH_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # # Set camera for the specific robot type (fetch)
        # self.scene.camera = CameraCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/head_tilt_link/head_camera",
        #     update_period=0.1,
        #     width=64,
        #     height=48,
        #     data_types=["rgb", "distance_to_image_plane"],
        #     spawn=sim_utils.PinholeCameraCfg(
        #         focal_length=24.0, focus_distance=400, horizontal_aperture=20.955, clipping_range=(0.5, 50)
        #     ),
        #     offset=CameraCfg.OffsetCfg(pos=(0.055, 0.02, 0.0225), rot=(0.5, -0.5, 0.5, -0.5)),
        # )

        # Set camera for the specific robot type (fetch)
        self.scene.camera = TiledCameraCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link/head_camera",
            update_period=0.1,
            width=64,
            height=48,
            data_types=["depth"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0, focus_distance=400, horizontal_aperture=20.955, clipping_range=(0.5, 50)
            ),
            offset=TiledCameraCfg.OffsetCfg(pos=(-0.0, 0.0, 1.2), rot=(0.386, -0.592, 0.592, -0.386)), # rot=(0.5, -0.5, 0.5, -0.5)),
        )

        # self.scene.ray_caster_camera = RayCasterCameraCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/head_tilt_link/head_camera",
        #     mesh_prim_paths=["/World/envs/envs_0/Cabinet"],
        #     # mesh_prim_paths=["/World/GroundPlane"],
        #     update_period=0.1,
        #     offset=RayCasterCameraCfg.OffsetCfg(pos=(0.055, 0.02, 0.0225), rot=(0,0,1,0)),
        #     data_types=["distance_to_image_plane", "normals", "distance_to_camera"],
        #     debug_vis=True,
        #     max_distance=10.0,
        #     pattern_cfg=patterns.PinholeCameraPatternCfg(
        #         focal_length=24.0,
        #         horizontal_aperture=20.955,
        #         height=480,
        #         width=640,
        #     ),
        # )


        # Defines the FrameTransformer senosr to add to the scene
        self.scene.camera_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link",
            debug_vis=True,
            visualizer_cfg=FRAME_MARKER_SMALL_CFG.replace(prim_path="/Visuals/CameraFrameTransformer"),
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/head_tilt_link",
                    name="camera",
                    offset=OffsetCfg(
                        pos=(0.055, 0.02, 0.0225),
                        rot=(0.5, -0.5, 0.5, -0.5),
                    ),
                )
            ],
        )

        # Set Actions for the specific robot type (franka)
        self.actions.body_joint_pos = mdp.JointPositionActionCfg(
            asset_name="robot",
            joint_names=["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"],
            # joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"],
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
                        # rot=(0.707, 0.0, 0.0, -0.707),
                        rot=(0.0, 0.707, 0.707, 0.0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/l_gripper_finger_link",
                    name="tool_leftfinger",
                    offset=OffsetCfg(
                        pos=(0.015, 0.01, 0.0),
                        # rot=(0.707, 0.0, 0.0, -0.707),
                        rot=(0.0, 0.707, 0.707, 0.0),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/r_gripper_finger_link",
                    name="tool_rightfinger",
                    offset=OffsetCfg(
                        pos=(0.015, -0.01, 0.0),
                        # rot=(0.707, 0.0, 0.0, -0.707),
                        rot=(0.0, 0.707, 0.707, 0.0),
                    ),
                ),
            ],
        )

        # override rewards
        self.rewards.approach_gripper_handle.params["offset"] = 0.04
        self.rewards.grasp_handle.params["open_joint_pos"] = 0.04
        self.rewards.grasp_handle.params["asset_cfg"].joint_names = ["r_gripper_finger_joint", "l_gripper_finger_joint"]

        # # override observations
        # self.observations.policy.image.params["cam_cfg"] = self.scene.camera


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
