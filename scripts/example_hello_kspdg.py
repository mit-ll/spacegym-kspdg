# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
This script is a "Hello World" for writing agents that can interact with
a KSPDG environment.

Instructions to Run:
- Start KSP game application.
- Select Start Game > Play Missions > Community Created > pe1_i3 > Continue
- In kRPC dialog box click Add server. Select Show advanced settings and select Auto-accept new clients. Then select Start Server
- In a terminal, run this script

"""

from kspdg.pe1.e1_envs import PE1_E1_I3_Env

# instantiate and reset the environment to populate game
env = PE1_E1_I3_Env()
env.reset()

# Environment automatically orients pursuer toward target
# therefore a niave pusuit policy to to simply burn at full
# thrust in pursuer's body-forward direction.
# Do this until the episode 
# (Do you think it can intercept even a non-maneuvering evader??)
is_done = False
act = [1.0, 0, 0, 1.0]  # forward throttle, right throttle, down throttle, duration [s]
while not is_done:
    obs, rew, is_done, info = env.step(act)

# printout info to see evaluation metrics of episode
print(info)

# close the environments to cleanup any processes
env.close()
