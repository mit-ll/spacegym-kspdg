# Copyright (c) 2022, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from datetime import datetime
from threading import Thread
import time
from types import SimpleNamespace
from typing import List

import gymnasium as gym
import krpc
import kspdg.utils.utils as U
import numpy as np

_INIT_LOADFILE = "20220516_PursuitEvade_init"
_MISSION_DONE_DIST_THRESHOLD = 20.0
_PURSUE_SWITCH_THRESHOLD_MOD = 0.66
_NEGATE_VEL_SWITCH_THRESHOLD = 0.03
_SWITCH_DELAY = 1


class EvasionEnvV20220714(gym.Env):
    """
    A simple pursuit-evasion orbital scenario

    Agent controls the evader and pursuer has a scripted policy

    The pursuit algorithm is very simplistic:
    the pursuer thrusts in the direction of the evader

    Objective is to avoid contact with the pursuer
    """

    # hard-coded, static parameters for pursuer vehicle
    # that are accessible yet constant (so shouldn't be
    # in observation which should really only be variable values)
    # Need for hard-coding rsc properties comes from the errors in
    # krpc's handling of thruster objects.
    PARAMS = SimpleNamespace()
    PARAMS.PURSUER = SimpleNamespace()
    PARAMS.PURSUER.RCS = SimpleNamespace()

    # Specific impulse assumes all RCS thrusters are identical RV-105
    # parts operating in vacuum
    PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE = 240  # [s]
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE = 1000  # [N]

    # Assumed number of thrusters creating thrust in each direction
    PARAMS.PURSUER.RCS.N_THRUSTERS_FORWARD = 8
    PARAMS.PURSUER.RCS.N_THRUSTERS_REVERSE = 8
    PARAMS.PURSUER.RCS.N_THRUSTERS_RIGHT = 4
    PARAMS.PURSUER.RCS.N_THRUSTERS_LEFT = 4
    PARAMS.PURSUER.RCS.N_THRUSTERS_UP = 4
    PARAMS.PURSUER.RCS.N_THRUSTERS_DOWN = 4

    # computed max thrust in each direction [N]
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_FORWARD = (
        PARAMS.PURSUER.RCS.N_THRUSTERS_FORWARD
        * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_REVERSE = (
        PARAMS.PURSUER.RCS.N_THRUSTERS_REVERSE
        * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_RIGHT = (
        PARAMS.PURSUER.RCS.N_THRUSTERS_RIGHT
        * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_LEFT = (
        PARAMS.PURSUER.RCS.N_THRUSTERS_LEFT
        * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_UP = (
        PARAMS.PURSUER.RCS.N_THRUSTERS_UP
        * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_DOWN = (
        PARAMS.PURSUER.RCS.N_THRUSTERS_DOWN
        * PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )

    # computed maximum fuel consumption rate in each direction [kg/s]
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_FORWARD = (
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_FORWARD
        / (U._G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_REVERSE = (
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_REVERSE
        / (U._G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_RIGHT = (
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_RIGHT
        / (U._G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_LEFT = (
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_LEFT
        / (U._G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_UP = (
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_UP
        / (U._G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.PURSUER.RCS.VACUUM_MAX_FUEL_CONSUMPTION_DOWN = (
        PARAMS.PURSUER.RCS.VACUUM_MAX_THRUST_DOWN
        / (U._G0 * PARAMS.PURSUER.RCS.VACUUM_SPECIFIC_IMPULSE)
    )

    def __init__(self):
        # establish observation space
        # TODO

        # establish action space (forward, up, right, time)
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, 0.0]), high=np.array([1.0, 1.0, 1.0, 10.0])
        )

        # call reset function
        self.reset()

    def reset(self):
        # remove prior connection and join prior pursuit thread
        if hasattr(self, "conn"):
            self.close()

        # establish krpc connect to send remote commands
        self.conn = krpc.connect(name="evasion_v20220516")
        print("Connected to kRPC server")

        # Load save file from start of mission scenario
        self.conn.space_center.load(_INIT_LOADFILE)

        # get vessel objects
        self.vesReferee, self.vesEvade, self.vesPursue = self.conn.space_center.vessels[
            :3
        ]

        # Set the pursuer as the active (i.e. human-controlled) vessel
        # and target evader
        self.conn.space_center.active_vessel = self.vesEvade
        self.conn.space_center.target_vessel = self.vesPursue
        print("Changing active vehicle...")
        time.sleep(1)  # give time to re-orient

        # set the evader to stability assist and orient in orbit-normal direction
        # orient pursuer in target-pointing direction

        self.vesEvade.control.sas = True
        self.vesPursue.control.sas = True
        print("Activating stability assist...")
        time.sleep(0.1)  # give time to re-orient
        self.vesPursue.control.sas_mode = self.vesPursue.control.sas_mode.normal
        self.vesEvade.control.sas_mode = self.vesEvade.control.sas_mode.normal
        print("Re-orienting Pursuer and Evader...")
        time.sleep(2)  # give time to re-orient

        # actuate RCS thrusters
        print("Activating reaction control systems...")
        self.vesEvade.control.rcs = True
        self.vesPursue.control.rcs = True

        # start process for pursuer maneuvers
        self.stop_pursue_thread = False
        self.negate_rel_v = False
        self.pursuer_thrust = [0, 0, 0]
        self.pursue_thread = Thread(target=self.pursue)
        self.pursue_thread.start()

        # start process for checking episode termination
        self.is_episode_done = False
        self.stop_episode_termination_thread = False
        self.episode_termination_thread = Thread(target=self.check_episode_termination)
        self.episode_termination_thread.start()

        return self.get_observation()

    def step(self, action):
        """Apply thrust and torque actuation for specified time
        Args:
            action : np.ndarray
                4-tuple of throttle values in 3D and timestep (forward, right, down, tstep)

        Ref:
            Actions are in forward, right, down to align with the right-handed version of the
            Vessel Surface Reference Frame
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
        """

        # parse and apply action
        self.vesEvade.control.forward = action[0]
        self.vesEvade.control.right = action[1]
        self.vesEvade.control.up = -action[2]

        # execute maneuver for specified time, checking for end
        # conditions while you do
        timeout = time.time() + action[3]
        while True:
            if self.is_episode_done or time.time() > timeout:
                break

        # zero out thrusts
        self.vesEvade.control.forward = 0.0
        self.vesEvade.control.up = 0.0
        self.vesEvade.control.right = 0.0

        # get observation
        obs = self.get_observation()

        # compute reward
        rew = -self.pursue_evade_relative_distance()

        info = {}

        return obs, rew, self.is_episode_done, info

    def convert_rhntw_to_rhpbody(self, v__rhntw: List[float]) -> List[float]:
        """Converts vector in right-handed NTW frame to pursuer vessel right-oriented body frame
        Args:
            v__ntw : List[float]
                3-vector represented in orbital NTW coords

        Returns
            v__rhpbody : List[float]
                3-vector vector represented in pursuer's right-hadded body coords (forward, right, down)

        Ref:
            Left-handed vessel body system:
                https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
            Right-handed NTW system: Vallado, 3rd Edition Sec 3.3.3
        """

        # convert right-handed NTW coords to left-handed NTW
        v__lhntw = U.convert_rhntw_to_lhntw(v__rhntw=v__rhntw)

        # convert left-handed NTW to left-handed vessel body coords
        # ref: https://krpc.github.io/krpc/python/api/space-center/space-center.html#SpaceCenter.transform_direction
        # ref: https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
        v__lhpbody = list(
            self.conn.space_center.transform_direction(
                direction=tuple(v__lhntw),
                from_=self.vesPursue.orbital_reference_frame,
                to=self.vesPursue.reference_frame,
            )
        )

        # convert left-handed body coords (right, forward, down) to right-handed body coords (forward, right, down)
        v__rhpbody = U.convert_lhbody_to_rhbody(v__lhbody=v__lhpbody)

        return v__rhpbody

    def get_observation(self):
        """return observation of pursuit and evader vessels from referee ref frame

        Returns:
            obs : list
                [0] : current vehicle (pursuer) mass [kg]
                [1] : current vehicle (pursuer) propellant  (mono prop) [kg]
                [2:5] : pursuer position in reference orbit NTW coords
                [5:8] : pursuer velocity in reference orbit NTW coords
                [8:11] : evader position in reference orbit NTW coords
                [11:14] : evader velocity in reference orbit NTW coords

        Ref:
            reference orbit coords from krpc are left-handed NTW frame(anti-radial, prograde, orbit-normal)
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-orbital-reference-frame
        """

        rf = self.vesReferee.orbital_reference_frame

        obs = []

        # get pursuer mass properties
        obs.append(self.vesPursue.mass)
        obs.append(self.vesPursue.resources.amount("MonoPropellant"))

        # got pursuer and evader position and velocity in
        # left-handed NTW frame relative to "referee" satellite
        p_p_r__lhntw = list(self.vesPursue.position(rf))
        v_p_r__lhntw = list(self.vesPursue.velocity(rf))
        p_e_r__lhntw = list(self.vesEvade.position(rf))
        v_e_r__lhntw = list(self.vesEvade.velocity(rf))

        # convert to right-hand system and add to observation
        p_p_r__rhntw = U.convert_lhntw_to_rhntw(p_p_r__lhntw)
        v_p_r__rhntw = U.convert_lhntw_to_rhntw(v_p_r__lhntw)
        p_e_r__rhntw = U.convert_lhntw_to_rhntw(p_e_r__lhntw)
        v_e_r__rhntw = U.convert_lhntw_to_rhntw(v_e_r__lhntw)

        # store observation of pursuer and evader position and velocity
        obs.extend(p_p_r__rhntw)
        obs.extend(v_p_r__rhntw)
        obs.extend(p_e_r__rhntw)
        obs.extend(v_e_r__rhntw)

        return obs

    def pursue_evade_relative_distance(self):
        """compute relative distance between pursuer and evader"""
        p_vesE_vesP__lhpbody = self.vesEvade.position(self.vesPursue.reference_frame)
        return np.linalg.norm(p_vesE_vesP__lhpbody)

    def check_episode_termination(self) -> bool:
        """determine if episode termination conditions are met

        Returns:
            bool
                true if episode termination criteria is met
        """

        while not self.stop_episode_termination_thread:
            # get distance to pursuer
            d_vesE_vesP = self.pursue_evade_relative_distance()
            self.is_episode_done = d_vesE_vesP < _MISSION_DONE_DIST_THRESHOLD
            if self.is_episode_done:
                print("Evasion Failed!")
                self.stop_episode_termination_thread = True
                self.stop_pursue_thread = True
                self.pursue_thread.join()

    def pursue(self):
        """use relative position of evader to thrust towards it
        Args:
            vesE : krpc.Vessel
                krpc vessel object for evader
            vesP : krpc.Vessel
                krpc vessel object for pursuer
        """
        d_vesE_vesP_switch = self.pursue_evade_relative_distance()
        d_vesE_vesP_prev = self.pursue_evade_relative_distance()
        time_switch = time.time()

        while not self.stop_pursue_thread:
            rf = self.vesReferee.orbital_reference_frame
            d_vesE_vesP_cur = self.pursue_evade_relative_distance()

            # mode 1: full thrust towards evader
            if not self.negate_rel_v:
                # get positions of evader and pursuer relative to referee
                p_vesE_vesR__lhntw = self.vesEvade.position(rf)
                p_vesP_vesR__lhntw = self.vesPursue.position(rf)

                # get position of evader relative to pursuer
                p_vesE_vesP__lhntw = (
                    p_vesE_vesR__lhntw[0] - p_vesP_vesR__lhntw[0],
                    p_vesE_vesR__lhntw[1] - p_vesP_vesR__lhntw[1],
                    p_vesE_vesR__lhntw[2] - p_vesP_vesR__lhntw[2],
                )

                # convert lhntw coordinates to lhpbody (centered around pursuer) coordinates
                p_vesE_vesP__lhpbody = self.conn.space_center.transform_direction(
                    direction=p_vesE_vesP__lhntw,
                    from_=rf,
                    to=self.vesPursue.reference_frame,
                )

                # find the largest component of the distance between evader and pursuer
                max_dist_comp = max(
                    abs(p_vesE_vesP__lhpbody[0]),
                    abs(p_vesE_vesP__lhpbody[1]),
                    abs(p_vesE_vesP__lhpbody[2]),
                )

                # always thrust in direction of evader at maximum speed
                self.vesPursue.control.right = p_vesE_vesP__lhpbody[0] / max_dist_comp
                self.vesPursue.control.forward = p_vesE_vesP__lhpbody[1] / max_dist_comp
                self.vesPursue.control.up = -p_vesE_vesP__lhpbody[2] / max_dist_comp

                # switch to equalizing relative velocity if the distance between the evader and pursuer is less than
                # _PURSUE_SWITCH_THRESHOLD_MOD times the distance at the last switch or the distance is greater than
                # in the previous iteration and more than a second has passed since the last switch
                activate_v_eq = d_vesE_vesP_cur <= (
                    d_vesE_vesP_switch * _PURSUE_SWITCH_THRESHOLD_MOD
                ) or (
                    (d_vesE_vesP_cur > d_vesE_vesP_prev)
                    and ((time.time() - time_switch) > _SWITCH_DELAY)
                )
                if activate_v_eq:
                    self.negate_rel_v = True
                    print("Equalizing velocities...")
            # mode 2: negate velocity relative to evader
            else:
                # get velocities of evader and pursuer relative to referee
                v_vesE_vesR__lhntw = self.vesEvade.velocity(rf)
                v_vesP_vesR__lhntw = self.vesPursue.velocity(rf)

                # get velocity of evader relative to pursuer
                v_vesE_vesP__lhntw = (
                    v_vesE_vesR__lhntw[0] - v_vesP_vesR__lhntw[0],
                    v_vesE_vesR__lhntw[1] - v_vesP_vesR__lhntw[1],
                    v_vesE_vesR__lhntw[2] - v_vesP_vesR__lhntw[2],
                )

                # convert lhntw coordinates to lhpbody (centered around pursuer) coordinates
                v_vesE_vesP__lhpbody = self.conn.space_center.transform_direction(
                    direction=v_vesE_vesP__lhntw,
                    from_=rf,
                    to=self.vesPursue.reference_frame,
                )

                # find the largest component of the difference in velocity between evader and pursuer
                max_v_diff_comp = max(
                    abs(v_vesE_vesP__lhpbody[0]),
                    abs(v_vesE_vesP__lhpbody[1]),
                    abs(v_vesE_vesP__lhpbody[2]),
                )

                # always thrust opposite direction of relative velocity vector maximum speed
                self.vesPursue.control.right = v_vesE_vesP__lhpbody[0] / max_v_diff_comp
                self.vesPursue.control.forward = (
                    v_vesE_vesP__lhpbody[1] / max_v_diff_comp
                )
                self.vesPursue.control.up = -v_vesE_vesP__lhpbody[2] / max_v_diff_comp

                # switch to thrusting towards evader if relative velocity between evader and pursuer is near 0
                v_diff = np.linalg.norm(v_vesE_vesP__lhpbody)
                # print(v_vesE_vesP__lhpbody)
                if abs(v_diff) <= _NEGATE_VEL_SWITCH_THRESHOLD:
                    d_vesE_vesP_switch = d_vesE_vesP_cur
                    time_switch = time.time()
                    self.negate_rel_v = False
                    print("Pursuing target...")

            d_vesE_vesP_prev = d_vesE_vesP_cur

            # # print thrust values for debugging
            # if self.vesPursue.control.forward != self.pursuer_thrust[0] or \
            #         self.vesPursue.control.forward != self.pursuer_thrust[1] or \
            #         self.vesPursue.control.forward != self.pursuer_thrust[2]:
            #     print("Changing pursuer thrust to ({right:.2f}, {forward:.2f}, {up:.2f})"
            #           .format(forward=self.vesPursue.control.forward,
            #                   right=self.vesPursue.control.right,
            #                   up=self.vesPursue.control.up))
            #
            #     self.pursuer_thrust[0] = self.vesPursue.control.forward
            #     self.pursuer_thrust[1] = self.vesPursue.control.right
            #     self.pursuer_thrust[2] = self.vesPursue.control.up

        print("Zeroing pursuer thrust...")
        self.vesPursue.control.forward = 0
        self.vesPursue.control.right = 0
        self.vesPursue.control.up = 0

    def close(self):
        # handle evasive maneuvering thread
        self.stop_pursue_thread = True
        self.pursue_thread.join()

        # handle episode termination thread
        self.stop_episode_termination_thread = True
        self.episode_termination_thread.join()

        # close connection to krpc server
        self.conn.close()
