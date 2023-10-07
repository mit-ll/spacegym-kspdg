# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.agent_api.base_agent import KSPDGBaseAgent


class NaivePursuitAgent(KSPDGBaseAgent):
    """An agent that naively burns directly toward it's target"""

    def __init__(self, **kwargs):
        super().__init__()

    def get_action(self, observation):
        """compute agent's action given observation

        This function is necessary to define as it overrides
        an abstract method
        """

        return [
            1.0,
            0,
            0,
            1.0,
        ]  # forward throttle, right throttle, down throttle, duration [s]
