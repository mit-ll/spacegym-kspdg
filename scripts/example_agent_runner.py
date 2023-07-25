# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

"""
This example shows how to define agents so that they can be systematically run
in a specific environment (important for agent evaluation purpuses)

Instructions to Run:
- Start KSP game application.
- Select Start Game > Play Missions > Community Created > pe1_i3 > Continue
- In kRPC dialog box click Add server. Select Show advanced settings and select Auto-accept new clients. Then select Start Server
- In a terminal, run this script

"""

from kspdg.pe1.e1_envs import PE1_E1_I3_Env
from kspdg.agent_api.runner import BaseAgentEnvRunner

class NaivePursuitAgent(BaseAgentEnvRunner):
    """An agent that naively burns directly toward it's target"""
    def __init__(self, runner_timeout:float, debug:bool=True):
        """
        Args:
            runner_timeout : float
                total time to run agent-environment pair
            debug : bool
                if true, set logging level to debug
        """
        super().__init__(
            env_cls=PE1_E1_I3_Env, 
            env_kwargs=None, 
            runner_timeout=runner_timeout,
            debug=debug)

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        return [1.0, 0, 0, 1.0]  # forward throttle, right throttle, down throttle, duration [s]

if __name__ == "__main__":
    naive_agent = NaivePursuitAgent(60)    # agent that will timeout after 200 seconds
    naive_agent.run()

