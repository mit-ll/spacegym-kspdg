# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.agent_api.base_agent import KSPDGBaseAgent

class FooAgent(KSPDGBaseAgent):
    def __init__(self, debug: bool = False):
        # super().__init__(logger_name=FooAgent.__name__, debug=debug)
        super().__init__(debug=debug)
    def get_action(self, observation):
        return None

def test_KSPDGBaseAgent_logger_0():
    """check logger is created and set to debug without error"""

    # ~~ ARRANGE ~~

    foo_agent = FooAgent(debug=True)

    # ~~ ACT ~~
    foo_agent.logger.warning("this is a warning statement")
    foo_agent.logger.info("this is an info statement")
    foo_agent.logger.debug("this is a debug statement")

    # ~~ ASSERT ~~~