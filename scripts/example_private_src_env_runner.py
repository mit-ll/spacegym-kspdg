# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
This example shows how to run a basic agent within a private-source environment w/ advanced bots

Instructions to Run:
- Start KSP game application.
- Select Start Game > Play Missions > Community Created > lbg1_i2 > Continue
- In kRPC dialog box click Add server. Select Show advanced settings and select Auto-accept new clients. Then select Start Server
- In a terminal, run this script

"""

from kspdg.agent_api.base_agent import KSPDGBaseAgent
from kspdg import LBG1_LG3_I2_V1
from kspdg.agent_api.runner import AgentEnvRunner

class NaivePursuitAgent(KSPDGBaseAgent):
    """An agent that naively burns directly toward it's target"""
    def __init__(self):
        super().__init__()

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        return {
            "burn_vec": [1.0, 0, 0, 1.0], # throttle in x-axis, throttle in y-axis, throttle in z-axis, duration [s]
            "ref_frame": 0  # burn_vec expressed in agent vessel's right-handed body frame. 
                            # i.e. forward throttle, right throttle, down throttle, 
                            # Can also use rhcbci (1) and rhntw (2) ref frames
        }

if __name__ == "__main__":
    naive_agent = NaivePursuitAgent()    
    runner = AgentEnvRunner(
        agent=naive_agent, 
        env_cls=LBG1_LG3_I2_V1, 
        env_kwargs=None,
        runner_timeout=100,     # agent runner that will timeout after 100 seconds
        debug=True,
        enable_telemetry=True)
    print(runner.run())

