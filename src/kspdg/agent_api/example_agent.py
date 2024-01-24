# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
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

        return {
            "burn_vec": [1.0, 0, 0, 1.0], # throttle in x-axis, throttle in y-axis, throttle in z-axis, duration [s]
            "vec_type": 0,  # burn vector as throttle values (if =1, burn vector represents thrust in [N])
            "ref_frame": 0  # burn_vec expressed in agent vessel's right-handed body frame. 
                            # i.e. forward throttle, right throttle, down throttle, 
                            # Can also use rhcbci (1) and rhntw (2) ref frames
        }
    
class PassivePursuitAgent(KSPDGBaseAgent):
    """An agent that does no burns"""
    def __init__(self, **kwargs):
        super().__init__()

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        return {"burn_vec": [0, 0, 0, 1.0], "vec_type": 0, "ref_frame": 0}
    
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

        return {"burn_vec": list(rnd_ctrl), "vec_type": 0, "ref_frame": 0}
    
class ProgradePursuitAgent(KSPDGBaseAgent):
    """An agent that just burns prograde"""
    def __init__(self, **kwargs):
        super().__init__()

    def get_action(self, observation):
        """ compute agent's action given observation

        This function is necessary to define as it overrides 
        an abstract method
        """

        return {
            "burn_vec": [0, 8000.0, 0, 1.0], 
            "vec_type": 1,  # burn vector represents thrust in [N]
            "ref_frame": 2}

    