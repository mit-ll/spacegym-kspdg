# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env

DEFAULT_EPISODE_TIMEOUT = 240.0 # [sec]

class SunBlockingGroup1Env(PursuitEvadeGroup1Env):
    '''
    Base environment for 1v1 sun-blocking orbital scenarios, which
    are direct variants of the pursuit-evasion environments
    
    All inherited classes share the following
        - Agent controls the pursuer and evader has a scripted policy (although specific policy varies)
        - Pursuer and evader vehicles are identical and are meant to be the same through all
        inherited scenarios (although there is not a current enforcement of this)
        - Observation and Action spaces shared between all child envs
    '''

    def __init__(self, loadfile:str, 
        episode_timeout:float = DEFAULT_EPISODE_TIMEOUT, 
        **kwargs):
        """
        Args:
            episode_timeout : float
                max length of episode [sec]
            capture_dist : float
                distance at which evader is considered captured [m]
        """

        super().__init__(
            loadfile=loadfile,
            episode_timeout=episode_timeout,
            capture_dist=None,
            **kwargs
        )

        # overwrite the pe1 observation space 
        # to include sun position information
        # (see get_observation for mapping)
        self.observation_space = NotImplemented
        
        # don't call reset. This allows instantiation and partial testing
        # without connecting to krpc server

    def _reset_episode_metrics(self) -> None:
        """ Reset attributes that track proximity, timing, and propellant use metrics
        """

        raise NotImplementedError()
    
    def get_reward(self) -> float:
        """ Compute reward value

        Reward is a function of evader-pursuer-sun angle and pursuer-evader distance
        
        Returns:
            rew : float
                reward at current step
        """
        raise NotImplementedError

    def get_info(self, observation: List, done: bool) -> Dict:
        """compute performance metrics
        Args:
            observation : List
                current / most recent observation of environment
            done : bool
                True if last step of episode
        """

        raise NotImplementedError

    def get_observation(self):
        ''' return observation of pursuit and evader vessels from referee ref frame

        Returns:
            obs : list
                [0] : mission elapsed time [s]
                [1] : current vehicle (pursuer) mass [kg]
                [2] : current vehicle (pursuer) propellant  (mono prop) [kg]
                [3:6] : pursuer position in reference orbit right-hand CBCI coords [m]
                [6:9] : pursuer velocity in reference orbit right-hand CBCI coords [m/s]
                [9:12] : evader position in reference orbit right-hand CBCI coords [m]
                [12:15] : evader velocity in reference orbit right-hand CBCI coords [m/s]

        Ref: 
            - CBCI stands for celestial-body-centered inertial which is a coralary to ECI coords
            (see notation: https://github.com/mit-ll/spacegym-kspdg#code-notation)
            - KSP's body-centered inertial reference frame is left-handed
            (see https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame)

        '''

        raise NotImplementedError