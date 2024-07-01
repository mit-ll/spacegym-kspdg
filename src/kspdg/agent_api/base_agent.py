# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import logging

from abc import ABC, abstractmethod
from kspdg.utils.loggers import create_logger

class KSPDGBaseAgent(ABC):

    def __init__(self, debug:bool=False) -> None:
        """
        Args:
            debug : bool
                if true, set logging config to debug
        """
        
        super().__init__()

        # create logger specific to agent 
        # (in contrast to environment logger and agent-env runner logger)
        # use agent child class name for more specificity on origin of log statements
        self.logger = create_logger(
            logger_name=self.__class__.__name__, 
            stream_log_level=logging.DEBUG if debug else logging.INFO)
        
    @abstractmethod
    def get_action(self, observation):
        raise NotImplementedError("Must be implemented by child class")