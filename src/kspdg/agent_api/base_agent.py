# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

from abc import ABC, abstractmethod


class KSPDGBaseAgent(ABC):
    @abstractmethod
    def get_action(self, observation):
        raise NotImplementedError("Must be implemented by child class")
