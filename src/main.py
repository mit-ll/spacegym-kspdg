import math
import time

from env.ksp_env import GameEnv
import krpc
import numpy as np
from stable_baselines3 import DQN

env = GameEnv(krpc.connect(name="Tracker"))

model = DQN("MlpPolicy", env, verbose=1)
# pip install stable-baselines3[extra]
model.learn(total_timesteps=10000, log_interval=4, progress_bar=True)
