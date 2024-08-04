# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Fetch robots.

The following configurations are available:

* :obj:`FETCH_PANDA_CFG`: Fetch robot
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg
from omni.isaac.lab_assets import ISAACLAB_ASSETS_DATA_DIR

##
# Configuration
##

FETCH_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/FetchRobot/Fetch/fetch.usd",
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        # collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # base
            "r_wheel_joint": 0.0,
            "l_wheel_joint": 0.0,
            "head_pan_joint": 0.0,
            "head_tilt_joint": 0.0,
            # fetch arm
            "torso_lift_joint": 0.0,
            "shoulder_pan_joint": -1.28,
            "shoulder_lift_joint": 1.51,
            "upperarm_roll_joint": 0.35,
            "elbow_flex_joint": 1.81,
            "forearm_roll_joint": 0.0,
            "wrist_flex_joint": 1.47,
            "wrist_roll_joint": 0.0,
            # "shoulder_pan_joint": 0.0,
            # "shoulder_lift_joint": 0.0,
            # "upperarm_roll_joint": 0.0,
            # "elbow_flex_joint": 0.0,
            # "forearm_roll_joint": 0.0,
            # "wrist_flex_joint": 0.0,
            # "wrist_roll_joint": 0.0,
            # # tool
            "r_gripper_finger_joint": 0.01,
            "l_gripper_finger_joint": 0.01,
        },
    ),
    actuators={
        "fetch_torso": ImplicitActuatorCfg(
            joint_names_expr=["torso_lift_joint"],
            effort_limit=10000.0,
            velocity_limit=0.1,
            stiffness=100000.0,
            damping=100000.0,
        ),
        "fetch_arm": ImplicitActuatorCfg(
            joint_names_expr=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "upperarm_roll_joint",
                "elbow_flex_joint",
                "forearm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
            ],
            effort_limit=10000.0,
            velocity_limit=1.0,
            stiffness=100000.0,
            damping=100000.0,
        ),
        "fetch_hand": ImplicitActuatorCfg(
            joint_names_expr=["r_gripper_finger_joint", "l_gripper_finger_joint"],
            effort_limit=1000.0,
            velocity_limit=0.05,
            stiffness=5000,
            damping=1000,
        ),
        "fetch_head": ImplicitActuatorCfg(
            joint_names_expr=["head_pan_joint", "head_tilt_joint"],
            effort_limit=1.0,
            velocity_limit=1.57,
            stiffness=100,
            damping=10,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Fetch robot."""