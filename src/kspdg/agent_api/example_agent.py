# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import numpy as np

from kspdg.agent_api.base_agent import KSPDGBaseAgent

class NaivePursuitAgent(KSPDGBaseAgent):
    """An agent that naively burns directly toward it's target"""
    def __init__(self, **kwargs):
        super().__init__()

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        return [1.0, 0, 0, 1.0]  # forward throttle, right throttle, down throttle, duration [s]
    
class PassivePursuitAgent(KSPDGBaseAgent):
    """An agent that does no burns"""
    def __init__(self, **kwargs):
        super().__init__()

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        return [0, 0, 0, 1.0]
    
class RandomPursuitAgent(KSPDGBaseAgent):
    """An agent that randomly burns in any direction"""
    def __init__(self, **kwargs):
        super().__init__()

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        rnd_thr = np.random.uniform(-1, 1, 3)  # Generate random body-axes throttle values
        rnd_ctrl = np.append(rnd_thr, np.random.uniform(0, 2))  # append random duration

        return list(rnd_ctrl)

    