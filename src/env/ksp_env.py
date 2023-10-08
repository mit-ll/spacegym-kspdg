"""
Other algorithms working on Gym should be easy to implement
Code could be done better, but the purpose was to make it similar to Open AI Gym environments
Written in Whiteaster by Piotr Kubica
"""

import datetime as dt
import math
import os
import time

import gymnasium as gym
from gymnasium import spaces
import numpy as np

CONTINUOUS = False


class GameEnv(gym.Env):
    def __init__(self, conn):
        self.set_telemetry(conn)
        self.pre_launch_setup()

        self.action_space = spaces.Discrete(9)  # -1, 0, 1 for all axis
        self.observation_space = spaces.Box(
            low=-1, high=1, shape=(6,), dtype=np.float32
        )

        # TODO(adrian) dict of dicts so we can unroll choose_action automatically
        self.available_actions = {
            "roll": (self.vessel.control.roll, (-1, 0, 1)),
            "pitch": (self.vessel.control.pitch, (-1, 0, 1)),
            "yaw": (self.vessel.control.yaw, (-1, 0, 1)),
        }

    def set_telemetry(self, conn):
        self.conn = conn
        self.vessel = conn.space_center.active_vessel

        # Setting up streams for telemetry
        self.ut = conn.add_stream(getattr, conn.space_center, "ut")
        self.altitude = conn.add_stream(getattr, self.vessel.flight(), "mean_altitude")
        self.apoapsis = conn.add_stream(getattr, self.vessel.orbit, "apoapsis_altitude")
        self.periapsis = conn.add_stream(
            getattr, self.vessel.orbit, "periapsis_altitude"
        )
        self.stage_2_resources = self.vessel.resources_in_decouple_stage(
            stage=2, cumulative=False
        )
        self.srb_fuel = conn.add_stream(self.stage_2_resources.amount, "SolidFuel")
        self.pitch = conn.add_stream(getattr, self.vessel.flight(), "pitch")
        self.heading = conn.add_stream(getattr, self.vessel.flight(), "heading")
        self.roll = conn.add_stream(getattr, self.vessel.flight(), "roll")
        self.g_force = conn.add_stream(getattr, self.vessel.flight(), "g_force")
        self.frame = self.vessel.orbit.body.reference_frame
        self.vert_speed = conn.add_stream(
            getattr, self.vessel.flight(self.frame), "vertical_speed"
        )
        self.speed = conn.add_stream(getattr, self.vessel.flight(), "velocity")
        self.lift = conn.add_stream(getattr, self.vessel.flight(), "lift")
        self.crew = conn.add_stream(getattr, self.vessel, "crew_count")
        self.parts = conn.add_stream(getattr, self.vessel.parts, "all")

    def pre_launch_setup(self):
        self.vessel.control.sas = False
        self.vessel.control.rcs = False

    def step(self, action):
        """
        possible continuous actions: yaw[-1:1], pitch[-1:1], roll[-1:1], throttle[0:1],
        other: forward[-1:1], up[-1:1], right[-1:1], wheel_throttle[-1:1], wheel_steering[-1:1],
        available observation
        https://krpc.github.io/krpc/python/api/space-center/control.html
        available states:
        https://krpc.github.io/krpc/python/api/space-center/flight.html
        https://krpc.github.io/krpc/python/api/space-center/orbit.html
        https://krpc.github.io/krpc/python/api/space-center/reference-frame.html
        :param action:
        :return state, reward, done, {}:
        """

        done = False

        action = action.tolist()

        start_act = self.ut()

        if CONTINUOUS:
            self.vessel.control.pitch = action[0]
            self.vessel.control.yaw = action[1]

        else:
            self.vessel.control.pitch = 0
            self.vessel.control.yaw = 0
            self.vessel.control.roll = 0

            self.choose_action(action)

        # 10 actions in one second in game time
        while self.ut() - start_act <= 0.1:
            continue

        state = self.get_state()

        reward = self.compute_reward()

        reward, done = self.epoch_ending(reward, done)

        self.conn.ui.message("Reward: " + str(round(reward, 2)), duration=0.5)

        return state, reward, done, done, {"mean_reward": reward}

    def choose_action(self, action):
        action_group = list(self.available_actions.keys())[action // 3]
        action_position_within_group = action % 3

        if action_group == "roll":
            self.vessel.control.roll = action_position_within_group

        elif action_group == "pitch":
            self.vessel.control.pitch = action_position_within_group

        elif action_group == "yaw":
            self.vessel.control.yaw = action_position_within_group

        self.conn.ui.message(
            f"{action=} {action_group=} {self.available_actions[action_group][1][action_position_within_group]}",
            duration=0.5,
        )

    def epoch_ending(self, reward, done):
        done = False

        return reward, done

    def reset(self, seed):
        """
        revivekerbals is a quick save file and should be in /GOG/KSP/game/saves/kill
        to run the code you will need to download it from
        https://drive.google.com/file/d/1L1DdeUdHpcMSmO8royVWocitVR93UwdE
        :param conn: krpc.connection
        :return: state
        """
        self.altitude_max = 0

        quick_save = "sat1"

        try:
            self.conn.space_center.load(quick_save)
        except Exception as ex:
            print("Error:", ex)
            print('Add "kill" save to your saves directory')
            exit("You have no quick save named {}. Terminating.".format(quick_save))

        time.sleep(3)

        # game is reloaded and we need to reset the telemetry
        self.set_telemetry(self.conn)
        self.pre_launch_setup()

        self.conn.space_center.physics_warp_factor = 0

        state = self.get_state()

        return state, {"conn": self.conn, "quicksave": "sat1"}

    def get_state(self):
        state = [
            math.sin(self.roll()),
            math.cos(self.roll()),
            math.sin(self.pitch()),
            math.cos(self.pitch()),
            math.sin(self.heading()),
            math.cos(self.heading()),
        ]

        return state

    def compute_reward(self):
        return -abs(self.roll()) / 180
