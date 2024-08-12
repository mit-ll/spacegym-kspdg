# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import logging
import sys

def create_logger(
        logger_name:str, 
        stream_log_level:int=logging.INFO,
        file_log:str=None,
        file_log_level:int=logging.WARNING):
    """ creates a logger with option for stdout and file
    
    Args:
        logger_name : str
            name of logger, typically __name__ of module creating logger
        stream_log_level:int
            level to log to stdout
        file_log : str | None
            name of logfile, if None, no log file created, only 
            logging to stdout
        file_log_level : int
            level to log to log file
    """
    logger = logging.getLogger(logger_name)

    # set logger level at the minimum between handlers
    logger.setLevel(min(stream_log_level, file_log_level))

    # Create handlers
    c_handler = logging.StreamHandler()
    c_handler.setLevel(stream_log_level)

    # Create formatters and add it to handlers
    c_format = logging.Formatter('%(asctime)s - %(levelname)s - %(name)s[%(process)d-%(threadName)s] - %(message)s')
    c_handler.setFormatter(c_format)

    # Add handlers to the logger
    logger.addHandler(c_handler)

    if file_log is not None:
        f_handler = logging.FileHandler('file.log')
        f_handler.setLevel(file_log_level)
        f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        f_handler.setFormatter(f_format)
        logger.addHandler(f_handler)

    return logger
