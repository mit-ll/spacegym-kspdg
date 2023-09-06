# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

'''
Test that require:
1. KSP to be running
2. Play Mission mode -> sb1_i5
3. kRPC server to be running
'''
import pytest
import time
import numpy as np

import kspdg.utils.constants as C
from kspdg.sb1.e1_envs import SB1_E1_I5_Env

@pytest.fixture
def sb1_e1_i5_env():
    '''setup and teardown of the PursuitEnvV20220516 object connected to kRPC server'''
    env = SB1_E1_I5_Env()
    env.reset()
    yield env
    env.close()

def test_get_reward_and_info_0(sb1_e1_i5_env):
    '''check that init state get info is as expected'''
    # ~~ ARRANGE ~~

    if sb1_e1_i5_env is None:
        env = SB1_E1_I5_Env()
        env.reset()
    else:
        env = sb1_e1_i5_env


    # ~~ ACT ~~ 
    rew = env.get_reward()
    info = env.get_info(None, False)

    # ~~ ASSERT ~~
    assert np.isclose(rew, 1.0, atol=1e-3)
    assert np.isclose(info[env.PARAMS.INFO.K_CUM_REWARD], 2.0, atol=1e-3)   # get info is called twice
    assert np.isclose(info[env.PARAMS.INFO.K_MAX_REWARD], 1.0, atol=1e-3)
    assert np.isclose(info[env.PARAMS.INFO.K_MIN_REWARD], 1.0, atol=1e-3)

    env.close()


if __name__ == "__main__":
    test_get_info_0(None)
