import glob
import math
from pathlib import Path
import time

from env.ksp_env import GameEnv
import krpc
import numpy as np
from stable_baselines3 import PPO
import wandb
from wandb.integration.sb3 import WandbCallback

run = wandb.init(
    project="detumbling",
    group="spaceapps-ksp-ai",
    monitor_gym=True,
    sync_tensorboard=True,
)

log_dir = list(glob.glob(f"./wandb/*-{run.id}"))
assert len(log_dir) == 1
log_dir = Path(log_dir[0])

env = GameEnv(krpc.connect(name="Tracker"), run)

model = PPO(
    "MlpPolicy",
    env,
    tensorboard_log=log_dir / "logs",
    learning_rate=0.01,
    n_steps=128,
    stats_window_size=2,
    device="cpu",
    verbose=1,
)

model._last_obs = None

model.learn(
    total_timesteps=1_000_000,
    log_interval=1,
    progress_bar=True,
    reset_num_timesteps=False,
    callback=WandbCallback(),
)
