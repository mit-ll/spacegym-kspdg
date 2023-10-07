# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Base class for Lady-Bandit-Guard Group 1 environments

from threading import Thread
import time
from types import SimpleNamespace
from typing import Dict, List

import gymnasium as gym
from kspdg.base_envs import KSPDGBaseEnv
import kspdg.utils.constants as C
import kspdg.utils.utils as U
import numpy as np
from numpy.typing import ArrayLike

DEFAULT_EPISODE_TIMEOUT = 240.0  # [sec]
DEFAULT_CAPTURE_DIST = 5.0  # [m]


class LadyBanditGuardGroup1Env(KSPDGBaseEnv):
    """
    Base environment for Lady-Bandit-Guard (LBG) Group 1 environments

    Nomenclature comes from paper:

    > Rusnak, Ilan. "The lady, the bandits and the body guards–a two team dynamic game."
    > IFAC Proceedings Volumes 38, no. 1 (2005): 441-446.

    All inherited classes share the following
        + There is 1 "Bandit", 1 "Guard", and 1 "Lady" spacecraft
        + Agent controls the Bandit spacecraft, a scripted bot(s) controls the
        Lady and the Guard (but the bot's policy varies between environments within the group)
        + Bandit and Guard have identical vehicle capabilities in each
        scenario, Lady vehicle may have the same or different vehicle capabilities
        + Lady initial orbit constant across all sencarios/envs; Bandit and Guard
        initial orbits are varied across environments in the group
        + Observation and Action spaces are constant across all scenarios/envs
    """

    # hard-coded, static parameters for lady, bandit, and guard vessels
    # that are accessible yet constant (so shouldn't be
    # in observation which should really only be variable values)
    # Need for hard-coding rsc properties comes from the errors in
    # krpc's handling of thruster objects.
    PARAMS = SimpleNamespace()
    PARAMS.LADY = SimpleNamespace()
    PARAMS.BANDIT = SimpleNamespace()
    PARAMS.GUARD = SimpleNamespace()
    PARAMS.BANDIT.RCS = SimpleNamespace()
    PARAMS.OBSERVATION = SimpleNamespace()
    PARAMS.INFO = SimpleNamespace()

    # observation space paramterization
    # [0] : mission elapsed time [s]
    # [1] : current vehicle (bandit) mass [kg]
    # [2] : current vehicle (bandit) propellant  (mono prop) [kg]
    # [3:6] : bandit position in reference orbit right-hand CBCI coords [m]
    # [6:9] : bandit velocity in reference orbit right-hand CBCI coords [m/s]
    # [9:12] : lady position in reference orbit right-hand CBCI coords [m]
    # [12:15] : lady velocity in reference orbit right-hand CBCI coords [m/s]
    # [15:18] : guard position in reference orbit right-hand CBCI coords [m]
    # [18:21] : guard velocity in reference orbit right-hand CBCI coords [m/s]
    PARAMS.OBSERVATION.LEN = 21
    PARAMS.OBSERVATION.I_MET = 0
    PARAMS.OBSERVATION.I_BANDIT_MASS = 1
    PARAMS.OBSERVATION.I_BANDIT_PROP_MASS = 2
    PARAMS.OBSERVATION.I_BANDIT_PX = 3
    PARAMS.OBSERVATION.I_BANDIT_PY = 4
    PARAMS.OBSERVATION.I_BANDIT_PZ = 5
    PARAMS.OBSERVATION.I_BANDIT_VX = 6
    PARAMS.OBSERVATION.I_BANDIT_VY = 7
    PARAMS.OBSERVATION.I_BANDIT_VZ = 8
    PARAMS.OBSERVATION.I_LADY_PX = 9
    PARAMS.OBSERVATION.I_LADY_PY = 10
    PARAMS.OBSERVATION.I_LADY_PZ = 11
    PARAMS.OBSERVATION.I_LADY_VX = 12
    PARAMS.OBSERVATION.I_LADY_VY = 13
    PARAMS.OBSERVATION.I_LADY_VZ = 14
    PARAMS.OBSERVATION.I_GUARD_PX = 15
    PARAMS.OBSERVATION.I_GUARD_PY = 16
    PARAMS.OBSERVATION.I_GUARD_PZ = 17
    PARAMS.OBSERVATION.I_GUARD_VX = 18
    PARAMS.OBSERVATION.I_GUARD_VY = 19
    PARAMS.OBSERVATION.I_GUARD_VZ = 20

    # # keys for observation fields
    # PARAMS.OBSERVATION.K_MET = "mission_elapsed_time"
    # PARAMS.OBSERVATION.K_BANDIT_MASS = "bandit_mass"
    # PARAMS.OBSERVATION.K_BANDIT_PROP_MASS = "bandit_propellant_mass"
    # PARAMS.OBSERVATION.K_BANDIT_PX = "posx_bandit_cb__rhcbci"
    # PARAMS.OBSERVATION.K_BANDIT_PY = "posy_bandit_cb__rhcbci"
    # PARAMS.OBSERVATION.K_BANDIT_PZ = "posz_bandit_cb__rhcbci"
    # PARAMS.OBSERVATION.K_BANDIT_VX = "velx_bandit_cb__rhcbci"
    # PARAMS.OBSERVATION.K_BANDIT_VY = "vely_bandit_cb__rhcbci"
    # PARAMS.OBSERVATION.K_BANDIT_VZ = "velz_bandit_cb__rhcbci"
    # PARAMS.OBSERVATION.K_LADY_PX = "posx_lady_cb__rhcbci"
    # PARAMS.OBSERVATION.K_LADY_PY = "posy_lady_cb__rhcbci"
    # PARAMS.OBSERVATION.K_LADY_PZ = "posz_lady_cb__rhcbci"
    # PARAMS.OBSERVATION.K_LADY_VX = "velx_lady_cb__rhcbci"
    # PARAMS.OBSERVATION.K_LADY_VY = "vely_lady_cb__rhcbci"
    # PARAMS.OBSERVATION.K_LADY_VZ = "velz_lady_cb__rhcbci"
    # PARAMS.OBSERVATION.K_GUARD_PX = "posx_guard_cb__rhcbci"
    # PARAMS.OBSERVATION.K_GUARD_PY = "posy_guard_cb__rhcbci"
    # PARAMS.OBSERVATION.K_GUARD_PZ = "posz_guard_cb__rhcbci"
    # PARAMS.OBSERVATION.K_GUARD_VX = "velx_guard_cb__rhcbci"
    # PARAMS.OBSERVATION.K_GUARD_VY = "vely_guard_cb__rhcbci"
    # PARAMS.OBSERVATION.K_GUARD_VZ = "velz_guard_cb__rhcbci"

    # info metric parameters
    PARAMS.INFO.K_CLOSEST_LB_APPROACH = "closest_lady_bandit_approach"
    PARAMS.INFO.K_CLOSEST_LB_APPROACH_TIME = "closest_lady_bandit_approach_time"
    PARAMS.INFO.K_CLOSEST_BG_APPROACH = "closest_bandit_guard_approach"
    PARAMS.INFO.K_MIN_LB_DISTSPEED_PRODUCT = (
        "minimum_lady_bandit_distance_speed_product"
    )
    PARAMS.INFO.K_BANDIT_FUEL_USAGE = "bandit_fuel_usage"
    PARAMS.INFO.K_LADY_FUEL_USAGE = "lady_fuel_usage"
    PARAMS.INFO.K_GUARD_FUEL_USAGE = "guard_fuel_usage"
    PARAMS.INFO.K_LB_DV_AT_TF = "expected_lady_bandit_deltav_at_final_time"

    # Specific impulse assumes all RCS thrusters are identical RV-105
    # parts operating in vacuum
    PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE = 240  # [s]
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE = 1000  # [N]

    # Assumed number of thrusters creating thrust in each direction
    PARAMS.BANDIT.RCS.N_THRUSTERS_FORWARD = 8
    PARAMS.BANDIT.RCS.N_THRUSTERS_REVERSE = 8
    PARAMS.BANDIT.RCS.N_THRUSTERS_RIGHT = 4
    PARAMS.BANDIT.RCS.N_THRUSTERS_LEFT = 4
    PARAMS.BANDIT.RCS.N_THRUSTERS_UP = 4
    PARAMS.BANDIT.RCS.N_THRUSTERS_DOWN = 4

    # computed max thrust in each direction [N]
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_FORWARD = (
        PARAMS.BANDIT.RCS.N_THRUSTERS_FORWARD
        * PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_REVERSE = (
        PARAMS.BANDIT.RCS.N_THRUSTERS_REVERSE
        * PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_RIGHT = (
        PARAMS.BANDIT.RCS.N_THRUSTERS_RIGHT
        * PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_LEFT = (
        PARAMS.BANDIT.RCS.N_THRUSTERS_LEFT
        * PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_UP = (
        PARAMS.BANDIT.RCS.N_THRUSTERS_UP
        * PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_DOWN = (
        PARAMS.BANDIT.RCS.N_THRUSTERS_DOWN
        * PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_PER_NOZZLE
    )

    # computed maximum fuel consumption rate in each direction [kg/s]
    PARAMS.BANDIT.RCS.VACUUM_MAX_FUEL_CONSUMPTION_FORWARD = (
        PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_FORWARD
        / (C.G0 * PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_FUEL_CONSUMPTION_REVERSE = (
        PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_REVERSE
        / (C.G0 * PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_FUEL_CONSUMPTION_RIGHT = (
        PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_RIGHT
        / (C.G0 * PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_FUEL_CONSUMPTION_LEFT = (
        PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_LEFT
        / (C.G0 * PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_FUEL_CONSUMPTION_UP = (
        PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_UP
        / (C.G0 * PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE)
    )
    PARAMS.BANDIT.RCS.VACUUM_MAX_FUEL_CONSUMPTION_DOWN = (
        PARAMS.BANDIT.RCS.VACUUM_MAX_THRUST_DOWN
        / (C.G0 * PARAMS.BANDIT.RCS.VACUUM_SPECIFIC_IMPULSE)
    )

    def __init__(
        self,
        loadfile: str,
        episode_timeout: float = DEFAULT_EPISODE_TIMEOUT,
        lady_capture_dist: float = DEFAULT_CAPTURE_DIST,
        bandit_capture_dist: float = DEFAULT_CAPTURE_DIST,
        **kwargs
    ):
        """
        Args:
            episode_timeout : float
                max length of episode [sec]
            lady_capture_dist : float
                distance at which lady is considered captured by bandit [m]
            bandit_capture_dist : float
                distance at which bandit is considered captured by guard [m]
        """

        super().__init__(**kwargs)

        assert episode_timeout > 0
        assert lady_capture_dist > 0
        assert bandit_capture_dist > 0
        self.episode_timeout = episode_timeout
        self.lady_capture_dist = lady_capture_dist
        self.bandit_capture_dist = bandit_capture_dist

        # establish load file for environment resets
        self.loadfile = loadfile

        # establish observation space (see get_observation for mapping)
        self.observation_space = gym.spaces.Box(
            low=np.concatenate((np.zeros(3), -np.inf * np.ones(18))),
            high=np.inf * np.ones(21),
        )
        assert self.observation_space.shape == (self.PARAMS.OBSERVATION.LEN,)

        # establish action space (forward, right, down, time) thrust of bandit
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, 0.0]), high=np.array([1.0, 1.0, 1.0, 10.0])
        )

        # don't call reset. This allows instantiation and partial testing
        # without connecting to krpc server

    def _reset_vessels(self):
        """Define vessel attirbutes and walkthrough initial configuration process"""

        # get vessel objects
        self.vesLady, self.vesBandit, self.vesGuard = self.conn.space_center.vessels[:4]

        # Set the bandit as the the active (i.e. human-controlled) vessel
        # and target lady
        print("Changing active vehicle to Bandit and setting target to Lady...")
        self.conn.space_center.active_vessel = self.vesBandit
        self.conn.space_center.target_vessel = self.vesLady
        time.sleep(0.5)  # give time to re-orient

        # orient bandit in target-pointing direction
        print("Activating Bandit SAS, RCS and orienting to Lady...")
        self.vesBandit.control.sas = True
        time.sleep(0.1)  # give time to re-orient
        self.vesBandit.control.sas_mode = self.vesBandit.control.sas_mode.target
        time.sleep(0.5)  # give time to re-orient

        # activate RCS thrusters
        self.vesBandit.control.rcs = True

    def _reset_episode_metrics(self) -> None:
        """Reset attributes that track proximity, timing, and propellant use metrics"""

        self.min_lb_dist = np.inf  # minimum distance between lady and bandit
        self.min_lb_dist_time = 0.0  # time of min dist between lady and bandit
        self.min_bg_dist = np.inf  # minimum distance between bandit and guard
        self.min_lb_distspeed_prod = (
            np.inf
        )  # minimum of distance-speed product between lady and bandit
        self.lady_init_mass = self.vesLady.mass
        self.bandit_init_mass = self.vesBandit.mass
        self.guard_init_mass = self.vesGuard.mass

    def _start_bot_threads(self) -> None:
        """Start parallel thread to continuously execute lady-guard policy"""

        self.stop_bot_thread = False

        # check that thread does not exist or is not running
        if hasattr(self, "bot_thread"):
            if self.bot_thread.is_alive():
                raise ConnectionError(
                    "bot_thread is already running."
                    + " Close and join bot_thread before restarting"
                )

        self.bot_thread = Thread(target=self.lady_guard_policy)
        self.bot_thread.start()

    def lady_guard_policy(self):
        """Behvaior policy to be continuously run by lady and guard vessels"""
        raise NotImplementedError("Must be implemented by child class")

    def step(self, action):
        """Apply thrust and torque actuation for specified time duration
        Args:
            action : np.ndarray
                4-tuple of throttle values in 3D and timestep (forward, right, down, tstep)

        Ref:
            Actions are in forward, right, down to align with the right-handed version of the
            Vessel Surface Reference Frame
            https://krpc.github.io/krpc/tutorials/reference-frames.html#vessel-surface-reference-frame
        """

        # parse and apply action
        self.vesBandit.control.forward = action[0]
        self.vesBandit.control.right = action[1]
        self.vesBandit.control.up = -action[2]

        # execute maneuver for specified time, checking for end
        # conditions while you do
        timeout = time.time() + action[3]
        while True:
            if self.is_episode_done or time.time() > timeout:
                break

        # zero out thrusts
        self.vesBandit.control.forward = 0.0
        self.vesBandit.control.up = 0.0
        self.vesBandit.control.right = 0.0

        # get observation
        obs = self.get_observation()

        # compute reward
        rew = -self.get_lb_relative_distance()

        # compute performance metrics
        info = self.get_info(obs, self.is_episode_done)

        return obs, rew, self.is_episode_done, info

    def get_info(self, observation: List, done: bool) -> Dict:
        """compute performance metrics
        Args:
            observation : List
                current / most recent observation of environment
            done : bool
                True if last step of episode
        """

        obs = observation
        info = dict()

        # parse banding and lady current states
        p0_b_cb__rhcbci = np.array(
            [
                obs[self.PARAMS.OBSERVATION.I_BANDIT_PX],
                obs[self.PARAMS.OBSERVATION.I_BANDIT_PY],
                obs[self.PARAMS.OBSERVATION.I_BANDIT_PZ],
            ]
        )
        v0_b_cb__rhcbci = np.array(
            [
                obs[self.PARAMS.OBSERVATION.I_BANDIT_VX],
                obs[self.PARAMS.OBSERVATION.I_BANDIT_VY],
                obs[self.PARAMS.OBSERVATION.I_BANDIT_VZ],
            ]
        )
        p0_l_cb__rhcbci = np.array(
            [
                obs[self.PARAMS.OBSERVATION.I_LADY_PX],
                obs[self.PARAMS.OBSERVATION.I_LADY_PY],
                obs[self.PARAMS.OBSERVATION.I_LADY_PZ],
            ]
        )
        v0_l_cb__rhcbci = np.array(
            [
                obs[self.PARAMS.OBSERVATION.I_LADY_VX],
                obs[self.PARAMS.OBSERVATION.I_LADY_VY],
                obs[self.PARAMS.OBSERVATION.I_LADY_VZ],
            ]
        )
        # p0_g_cb__rhcbci = np.array([
        #     obs[self.PARAMS.OBSERVATION.I_GUARD_PX],
        #     obs[self.PARAMS.OBSERVATION.I_GUARD_PY],
        #     obs[self.PARAMS.OBSERVATION.I_GUARD_PZ],
        # ])
        # v0_g_cb__rhcbci = np.array([
        #     obs[self.PARAMS.OBSERVATION.I_GUARD_VX],
        #     obs[self.PARAMS.OBSERVATION.I_GUARD_VY],
        #     obs[self.PARAMS.OBSERVATION.I_GUARD_VZ],
        # ])

        # nearest approach between lady and bandit
        # d_vesL_vesB = np.linalg.norm(p0_b_cb__rhcbci-p0_l_cb__rhcbci)
        d_vesL_vesB = self.get_lb_relative_distance()
        if d_vesL_vesB < self.min_lb_dist:
            self.min_lb_dist = d_vesL_vesB
            self.min_lb_dist_time = obs[self.PARAMS.OBSERVATION.I_MET]
        info[self.PARAMS.INFO.K_CLOSEST_LB_APPROACH] = self.min_lb_dist
        info[self.PARAMS.INFO.K_CLOSEST_LB_APPROACH_TIME] = self.min_lb_dist_time

        # nearest approach between bandit and guard
        # d_vesB_vesG = np.linalg.norm(p0_g_cb__rhcbci-p0_b_cb__rhcbci)
        d_vesB_vesG = self.get_bg_relative_distance()
        if d_vesB_vesG < self.min_bg_dist:
            self.min_bg_dist = d_vesB_vesG
        info[self.PARAMS.INFO.K_CLOSEST_BG_APPROACH] = self.min_bg_dist

        # distance-speed product metric
        s_vesL_vesB = np.linalg.norm(v0_b_cb__rhcbci - v0_l_cb__rhcbci)
        lb_ds_prod = d_vesL_vesB * s_vesL_vesB
        if lb_ds_prod < self.min_lb_distspeed_prod:
            self.min_lb_distspeed_prod = lb_ds_prod
        info[self.PARAMS.INFO.K_MIN_LB_DISTSPEED_PRODUCT] = self.min_lb_distspeed_prod

        # fuel usage
        info[self.PARAMS.INFO.K_BANDIT_FUEL_USAGE] = (
            self.bandit_init_mass - self.vesBandit.mass
        )
        info[self.PARAMS.INFO.K_LADY_FUEL_USAGE] = (
            self.lady_init_mass - self.vesLady.mass
        )
        info[self.PARAMS.INFO.K_GUARD_FUEL_USAGE] = (
            self.guard_init_mass - self.vesGuard.mass
        )

        # compute approximate delta-v need to intercept non-maneuvering lady
        if done:
            # call capture dv estimator
            dv0, dvf = U.estimate_capture_dv(
                p0_prs=p0_b_cb__rhcbci,
                v0_prs=v0_b_cb__rhcbci,
                p0_evd=p0_l_cb__rhcbci,
                v0_evd=v0_l_cb__rhcbci,
                tof=self.episode_timeout,
            )

            info[self.PARAMS.INFO.K_LB_DV_AT_TF] = dv0 + dvf

        else:
            info[self.PARAMS.INFO.K_LB_DV_AT_TF] = None

        return info

    def get_observation(self) -> ArrayLike:
        """return observation of bandit, lady, and guard pos-vel state

        Returns:
            obs : ArrayLike
                [0] : mission elapsed time [s]
                [1] : current vehicle (bandit) mass [kg]
                [2] : current vehicle (bandit) propellant  (mono prop) [kg]
                [3:6] : bandit position wrt CB in right-hand CBCI coords [m]
                [6:9] : bandit velocity wrt CB in right-hand CBCI coords [m/s]
                [9:12] : lady position wrt CB in right-hand CBCI coords [m]
                [12:15] : lady velocity wrt CB in right-hand CBCI coords [m/s]
                [15:18] : guard position wrt CB in right-hand CBCI coords [m]
                [18:21] : guard velocity wrt CB in right-hand CBCI coords [m/s]

        Ref:
            - CBCI stands for celestial-body-centered inertial which is a coralary to ECI coords
            (see notation: https://github.com/mit-ll/spacegym-kspdg#code-notation)
            - KSP's body-centered inertial reference frame is left-handed
            (see https://krpc.github.io/krpc/python/api/space-center/celestial-body.html#SpaceCenter.CelestialBody.non_rotating_reference_frame)

        """

        rf = self.vesBandit.orbit.body.non_rotating_reference_frame

        obs = [None] * self.PARAMS.OBSERVATION.LEN

        # get bandit (i.e. vehicle under control) mass properties
        obs[self.PARAMS.OBSERVATION.I_MET] = self.vesBandit.met
        obs[self.PARAMS.OBSERVATION.I_BANDIT_MASS] = self.vesBandit.mass
        obs[
            self.PARAMS.OBSERVATION.I_BANDIT_PROP_MASS
        ] = self.vesBandit.resources.amount("MonoPropellant")

        # get bandit, lady, and guard position and velocity in
        # left-handed celestial-body-centered-inertial frame
        p_b_cb__lhcbci = list(self.vesBandit.position(rf))
        v_b_cb__lhcbci = list(self.vesBandit.velocity(rf))
        p_l_cb__lhcbci = list(self.vesLady.position(rf))
        v_l_cb__lhcbci = list(self.vesLady.velocity(rf))
        p_g_cb__lhcbci = list(self.vesGuard.position(rf))
        v_g_cb__lhcbci = list(self.vesGuard.velocity(rf))

        # convert to right-hand system and add to observation
        p_b_cb__rhcbci = U.convert_lhcbci_to_rhcbci(p_b_cb__lhcbci)
        v_b_cb__rhcbci = U.convert_lhcbci_to_rhcbci(v_b_cb__lhcbci)
        p_l_cb__rhcbci = U.convert_lhcbci_to_rhcbci(p_l_cb__lhcbci)
        v_l_cb__rhcbci = U.convert_lhcbci_to_rhcbci(v_l_cb__lhcbci)
        p_g_cb__rhcbci = U.convert_lhcbci_to_rhcbci(p_g_cb__lhcbci)
        v_g_cb__rhcbci = U.convert_lhcbci_to_rhcbci(v_g_cb__lhcbci)

        # store observation of bandit, lady, and guard position and velocity
        (
            obs[self.PARAMS.OBSERVATION.I_BANDIT_PX],
            obs[self.PARAMS.OBSERVATION.I_BANDIT_PY],
            obs[self.PARAMS.OBSERVATION.I_BANDIT_PZ],
        ) = p_b_cb__rhcbci

        (
            obs[self.PARAMS.OBSERVATION.I_BANDIT_VX],
            obs[self.PARAMS.OBSERVATION.I_BANDIT_VY],
            obs[self.PARAMS.OBSERVATION.I_BANDIT_VZ],
        ) = v_b_cb__rhcbci

        (
            obs[self.PARAMS.OBSERVATION.I_LADY_PX],
            obs[self.PARAMS.OBSERVATION.I_LADY_PY],
            obs[self.PARAMS.OBSERVATION.I_LADY_PZ],
        ) = p_l_cb__rhcbci

        (
            obs[self.PARAMS.OBSERVATION.I_LADY_VX],
            obs[self.PARAMS.OBSERVATION.I_LADY_VY],
            obs[self.PARAMS.OBSERVATION.I_LADY_VZ],
        ) = v_l_cb__rhcbci

        (
            obs[self.PARAMS.OBSERVATION.I_GUARD_PX],
            obs[self.PARAMS.OBSERVATION.I_GUARD_PY],
            obs[self.PARAMS.OBSERVATION.I_GUARD_PZ],
        ) = p_g_cb__rhcbci

        (
            obs[self.PARAMS.OBSERVATION.I_GUARD_VX],
            obs[self.PARAMS.OBSERVATION.I_GUARD_VY],
            obs[self.PARAMS.OBSERVATION.I_GUARD_VZ],
        ) = v_g_cb__rhcbci

        return np.array(obs)

    def get_lb_relative_distance(self):
        """compute relative distance between lady and bandit"""
        p_vesL_vesB__lhpbody = self.vesLady.position(self.vesBandit.reference_frame)
        return np.linalg.norm(p_vesL_vesB__lhpbody)

    def get_bg_relative_distance(self):
        """compute relative distance between bandit and guard"""
        p_vesB_vesG__lhpbody = self.vesBandit.position(self.vesGuard.reference_frame)
        return np.linalg.norm(p_vesB_vesG__lhpbody)

    def enforce_episode_termination(self):
        """determine if distance or timeout episode termination conditions are met"""

        while not self.stop_episode_termination_thread:
            # check termination condition: lady-bandit proximity
            d_vesL_vesB = self.get_lb_relative_distance()
            is_lady_captured = d_vesL_vesB < self.lady_capture_dist
            if is_lady_captured:
                print("\n~~~SUCCESS! LADY CAPTURED BY BANDIT~~~\n")

            # check termination condition: bandit-guard proximity
            d_vesB_vesG = self.get_bg_relative_distance()
            is_bandit_captured = d_vesB_vesG < self.bandit_capture_dist
            if is_bandit_captured:
                print("\n~~~FAILURE! BANDIT CAPTURED BY GUARD~~~\n")

            # check for episode timeout
            is_timeout = self.vesBandit.met > self.episode_timeout
            if is_timeout:
                print("\n~~~EPISODE TIMEOUT~~~\n")

            if is_lady_captured or is_bandit_captured or is_timeout:
                print("Terminating episode...\n")
                self.is_episode_done = True
                self.stop_bot_thread = True
                self.stop_episode_termination_thread = True

    def close(self):
        # handle evasive maneuvering thread
        self.stop_bot_thread = True
        self.bot_thread.join()

        # handle episode termination thread
        self.stop_episode_termination_thread = True
        self.episode_termination_thread.join()

        # close connection to krpc server
        self.conn.close()

    # @classmethod
    # def observation_list_to_dict(cls, obs_list):
    #     """convert observation from list to dict"""
    #     assert len(obs_list) == cls.PARAMS.OBSERVATION.LEN
    #     obs_dict = dict()
    #     obs_dict[cls.PARAMS.OBSERVATION.K_MET] = obs_list[cls.PARAMS.OBSERVATION.I_MET]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_MASS] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_MASS]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PROP_MASS] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PROP_MASS]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PX] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PX]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PY] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PY]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PZ] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PZ]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_VX] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_VX]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_VY] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_VY]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_VZ] = obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_VZ]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_LADY_PX] = obs_list[cls.PARAMS.OBSERVATION.I_LADY_PX]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_LADY_PY] = obs_list[cls.PARAMS.OBSERVATION.I_LADY_PY]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_LADY_PZ] = obs_list[cls.PARAMS.OBSERVATION.I_LADY_PZ]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_LADY_VX] = obs_list[cls.PARAMS.OBSERVATION.I_LADY_VX]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_LADY_VY] = obs_list[cls.PARAMS.OBSERVATION.I_LADY_VY]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_LADY_VZ] = obs_list[cls.PARAMS.OBSERVATION.I_LADY_VZ]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_PX] = obs_list[cls.PARAMS.OBSERVATION.I_GUARD_PX]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_PY] = obs_list[cls.PARAMS.OBSERVATION.I_GUARD_PY]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_PZ] = obs_list[cls.PARAMS.OBSERVATION.I_GUARD_PZ]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_VX] = obs_list[cls.PARAMS.OBSERVATION.I_GUARD_VX]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_VY] = obs_list[cls.PARAMS.OBSERVATION.I_GUARD_VY]
    #     obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_VZ] = obs_list[cls.PARAMS.OBSERVATION.I_GUARD_VZ]

    #     return obs_dict

    # @classmethod
    # def observation_dict_to_list(cls, obs_dict):
    #     """convert observation from list to dict"""
    #     obs_list = cls.PARAMS.OBSERVATION.LEN * [None]
    #     obs_list[cls.PARAMS.OBSERVATION.I_MET] = obs_dict[cls.PARAMS.OBSERVATION.K_MET]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_MASS] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_MASS]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PROP_MASS] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PROP_MASS]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PX] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PX]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PY] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PY]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_PZ] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_PZ]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_VX] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_VX]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_VY] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_VY]
    #     obs_list[cls.PARAMS.OBSERVATION.I_BANDIT_VZ] = obs_dict[cls.PARAMS.OBSERVATION.K_BANDIT_VZ]
    #     obs_list[cls.PARAMS.OBSERVATION.I_LADY_PX] = obs_dict[cls.PARAMS.OBSERVATION.K_LADY_PX]
    #     obs_list[cls.PARAMS.OBSERVATION.I_LADY_PY] = obs_dict[cls.PARAMS.OBSERVATION.K_LADY_PY]
    #     obs_list[cls.PARAMS.OBSERVATION.I_LADY_PZ] = obs_dict[cls.PARAMS.OBSERVATION.K_LADY_PZ]
    #     obs_list[cls.PARAMS.OBSERVATION.I_LADY_VX] = obs_dict[cls.PARAMS.OBSERVATION.K_LADY_VX]
    #     obs_list[cls.PARAMS.OBSERVATION.I_LADY_VY] = obs_dict[cls.PARAMS.OBSERVATION.K_LADY_VY]
    #     obs_list[cls.PARAMS.OBSERVATION.I_LADY_VZ] = obs_dict[cls.PARAMS.OBSERVATION.K_LADY_VZ]
    #     obs_list[cls.PARAMS.OBSERVATION.I_GUARD_PX] = obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_PX]
    #     obs_list[cls.PARAMS.OBSERVATION.I_GUARD_PY] = obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_PY]
    #     obs_list[cls.PARAMS.OBSERVATION.I_GUARD_PZ] = obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_PZ]
    #     obs_list[cls.PARAMS.OBSERVATION.I_GUARD_VX] = obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_VX]
    #     obs_list[cls.PARAMS.OBSERVATION.I_GUARD_VY] = obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_VY]
    #     obs_list[cls.PARAMS.OBSERVATION.I_GUARD_VZ] = obs_dict[cls.PARAMS.OBSERVATION.K_GUARD_VZ]

    #     return obs_list
