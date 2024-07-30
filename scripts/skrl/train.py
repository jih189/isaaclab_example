# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to train RL agent with skrl.

Visit the skrl documentation (https://skrl.readthedocs.io) to see the examples structured in
a more user-friendly way.
"""

"""Launch Isaac Sim Simulator first."""


import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with skrl.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--distributed", action="store_true", default=False, help="Run training with multiple GPUs or nodes."
)
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
parser.add_argument(
    "--ml_framework",
    type=str,
    default="torch",
    choices=["torch", "jax", "jax-numpy"],
    help="The ML framework used for training the skrl agent.",
)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

if args_cli.video:
    args_cli.enable_cameras = True

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
from datetime import datetime

import skrl
from skrl.utils import set_seed

if args_cli.ml_framework.startswith("torch"):
    from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
    from skrl.memories.torch import RandomMemory
    from skrl.trainers.torch import SequentialTrainer
    from skrl.utils.model_instantiators.torch import deterministic_model, gaussian_model, shared_model
elif args_cli.ml_framework.startswith("jax"):
    from skrl.agents.jax.ppo import PPO, PPO_DEFAULT_CONFIG
    from skrl.memories.jax import RandomMemory
    from skrl.trainers.jax import SequentialTrainer
    from skrl.utils.model_instantiators.jax import deterministic_model, gaussian_model

from omni.isaac.lab.utils.dict import print_dict
from omni.isaac.lab.utils.io import dump_pickle, dump_yaml

# import omni.isaac.lab_tasks  # noqa: F401
import fetch_project.tasks
from omni.isaac.lab_tasks.utils import load_cfg_from_registry, parse_env_cfg
from omni.isaac.lab_tasks.utils.wrappers.skrl import SkrlVecEnvWrapper, process_skrl_cfg


def main():
    """Train with skrl agent."""
    # configure the ML framework into the global skrl variable
    if args_cli.ml_framework.startswith("jax"):
        skrl.config.jax.backend = "jax" if args_cli.ml_framework == "jax" else "numpy"

    # read the seed from command line
    args_cli_seed = args_cli.seed

    # parse configuration
    env_cfg = parse_env_cfg(
        args_cli.task, use_gpu=not args_cli.cpu, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    print("check task: ", args_cli.task)
    experiment_cfg = load_cfg_from_registry(args_cli.task, "skrl_cfg_entry_point")

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "skrl", experiment_cfg["agent"]["experiment"]["directory"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Logging experiment in directory: {log_root_path}")
    # specify directory for logging runs: {time-stamp}_{run_name}
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if experiment_cfg["agent"]["experiment"]["experiment_name"]:
        log_dir += f'_{experiment_cfg["agent"]["experiment"]["experiment_name"]}'
    # set directory into agent config
    experiment_cfg["agent"]["experiment"]["directory"] = log_root_path
    experiment_cfg["agent"]["experiment"]["experiment_name"] = log_dir
    # update log_dir
    log_dir = os.path.join(log_root_path, log_dir)

    # multi-gpu training config
    if args_cli.distributed:
        if args_cli.ml_framework.startswith("jax"):
            raise ValueError("Multi-GPU distributed training not yet supported in JAX")
        # update env config device
        env_cfg.sim.device = f"cuda:{app_launcher.local_rank}"

    # max iterations for training
    if args_cli.max_iterations:
        experiment_cfg["trainer"]["timesteps"] = args_cli.max_iterations * experiment_cfg["agent"]["rollouts"]

    # dump the configuration into log-directory
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), experiment_cfg)
    dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
    dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), experiment_cfg)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)
    # wrap around environment for skrl
    env = SkrlVecEnvWrapper(env, ml_framework=args_cli.ml_framework)  # same as: `wrap_env(env, wrapper="isaaclab")`

    # set seed for the experiment (override from command line)
    set_seed(args_cli_seed if args_cli_seed is not None else experiment_cfg["seed"])

    # instantiate models using skrl model instantiator utility
    # https://skrl.readthedocs.io/en/latest/api/utils/model_instantiators.html
    models = {}
    if args_cli.ml_framework.startswith("jax"):
        experiment_cfg["models"]["separate"] = True  # shared model is not supported in JAX
    # non-shared models
    if experiment_cfg["models"]["separate"]:
        models["policy"] = gaussian_model(
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=env.device,
            **process_skrl_cfg(experiment_cfg["models"]["policy"], ml_framework=args_cli.ml_framework),
        )
        models["value"] = deterministic_model(
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=env.device,
            **process_skrl_cfg(experiment_cfg["models"]["value"], ml_framework=args_cli.ml_framework),
        )
    # shared models
    else:
        models["policy"] = shared_model(
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=env.device,
            structure=None,
            roles=["policy", "value"],
            parameters=[
                process_skrl_cfg(experiment_cfg["models"]["policy"], ml_framework=args_cli.ml_framework),
                process_skrl_cfg(experiment_cfg["models"]["value"], ml_framework=args_cli.ml_framework),
            ],
        )
        models["value"] = models["policy"]
    # instantiate models' state dict
    if args_cli.ml_framework.startswith("jax"):
        for role, model in models.items():
            model.init_state_dict(role)

    # instantiate a RandomMemory as rollout buffer (any memory can be used for this)
    # https://skrl.readthedocs.io/en/latest/api/memories/random.html
    memory_size = experiment_cfg["agent"]["rollouts"]  # memory_size is the agent's number of rollouts
    memory = RandomMemory(memory_size=memory_size, num_envs=env.num_envs, device=env.device)

    # configure and instantiate PPO agent
    # https://skrl.readthedocs.io/en/latest/api/agents/ppo.html
    agent_cfg = PPO_DEFAULT_CONFIG.copy()
    experiment_cfg["agent"]["rewards_shaper"] = None  # avoid 'dictionary changed size during iteration'
    agent_cfg.update(process_skrl_cfg(experiment_cfg["agent"], ml_framework=args_cli.ml_framework))

    agent_cfg["state_preprocessor_kwargs"].update({"size": env.observation_space, "device": env.device})
    agent_cfg["value_preprocessor_kwargs"].update({"size": 1, "device": env.device})

    agent = PPO(
        models=models,
        memory=memory,
        cfg=agent_cfg,
        observation_space=env.observation_space,
        action_space=env.action_space,
        device=env.device,
    )

    # configure and instantiate a custom RL trainer for logging episode events
    # https://skrl.readthedocs.io/en/latest/api/trainers.html
    trainer_cfg = experiment_cfg["trainer"]
    trainer_cfg["close_environment_at_exit"] = False
    trainer = SequentialTrainer(cfg=trainer_cfg, env=env, agents=agent)

    # train the agent
    trainer.train()

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
