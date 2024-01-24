# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from kspdg.pe1.pe1_base import PursuitEvadeGroup1Env

DEFAULT_EVADE_DIST = 200.0   # [m]
DEFAULT_EVADE_THROTTLE = 1.0

class PE1_E3_ParentEnv(PursuitEvadeGroup1Env):
    def __init__(self, loadfile: str, 
        evade_dist:float = DEFAULT_EVADE_DIST, 
        evade_throttle:float = DEFAULT_EVADE_THROTTLE,
        **kwargs):
        """
        Args
            evade_dist : float
                distance from pursuer in which evader starts 
                evasive maneuvers [m]
            evade_throttle : float
                throttle value to use for evasion [0-1]
        """
        super().__init__(loadfile=loadfile, **kwargs)
        assert evade_dist > 0.0
        assert evade_throttle >= 0.0
        assert evade_throttle <= 1.0
        self.evade_dist = evade_dist
        self.evade_throttle = evade_throttle

    def evasive_maneuvers(self):
        '''use relative position of pursuer to execute simple evasive maneuver
        '''
        was_evading = False
        is_control_set = False
        while not self.stop_bot_thread:


            # get distance to pursuer
            d_vesE_vesP = self.get_pe_relative_distance()

            # check for control range
            if d_vesE_vesP < self.PARAMS.EVADER.CONTROL_RANGE:

                # turn on evader sas if not yet active
                if not is_control_set:
                    self.logger.info("Activating Evader SAS and RCS...")
                    self.vesEvade.control.sas = True
                    self.vesEvade.control.sas_mode = self.vesEvade.control.sas_mode.normal
                    self.vesEvade.control.rcs = True
                    is_control_set = True

                # if pursuer is too close, evade in orbit-normal direction
                if d_vesE_vesP < self.evade_dist:
                    self.vesEvade.control.forward = self.evade_throttle
                    if not was_evading:
                        self.logger.info("\n~~~PURSUER DETECTED! Executing evasive maneuvers~~~\n")
                    was_evading = True
                else:
                    self.vesEvade.control.forward = 0.0
                    if was_evading:
                        self.logger.info("\n~~~NO PURSUER IN RANGE! Zeroing thrust...~~~\n")
                    was_evading = False
        
        # terminate throttle
        if self.get_pe_relative_distance() < self.PARAMS.EVADER.CONTROL_RANGE:
            self.vesEvade.control.forward = 0.0
            self.vesEvade.control.right = 0.0
            self.vesEvade.control.up = 0.0

class PE1_E3_I1_Env(PE1_E3_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I1, **kwargs)
    
class PE1_E3_I2_Env(PE1_E3_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I2, **kwargs)

class PE1_E3_I3_Env(PE1_E3_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I3, **kwargs)

class PE1_E3_I4_Env(PE1_E3_ParentEnv):
    def __init__(self, **kwargs):
        super().__init__(loadfile=PursuitEvadeGroup1Env.LOADFILE_I4, **kwargs)

class PE1_E3_I20220516_Env(PE1_E3_ParentEnv):

    INIT_LOADFILE = "20220516_PursuitEvade_init"
    MISSION_DONE_DIST_THRESHOLD = 20.0     # [m]

    def __init__(self):
        super().__init__(loadfile=PE1_E3_I20220516_Env.INIT_LOADFILE)

    def check_episode_termination(self) -> bool:
        '''determine if episode termination conditions are met
        
        Returns:
            bool
                true if episode termination criteria is met
        '''

        while not self.stop_episode_termination_thread:
            # get distance to pursuer
            d_vesE_vesP = self.get_pe_relative_distance()
            self.is_episode_done = d_vesE_vesP < PE1_E3_I20220516_Env.MISSION_DONE_DIST_THRESHOLD
            if self.is_episode_done:
                self.logger.info("\n~~~SUCCESSFUL CAPTURE!~~~\n")
                self.stop_episode_termination_thread = True