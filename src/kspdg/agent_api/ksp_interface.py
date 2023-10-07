# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import logging

from kspdg.utils.loggers import create_logger


def ksp_interface_loop(
    env_cls,
    env_kwargs,
    obs_conn_send,
    act_conn_recv,
    termination_event,
    observation_query_event,
    return_dict,
    debug,
):
    """Parallel process for running and interacting with environment

    This loop creates a separate process so that environment and agent (policy) can be run concurrently
    It handles KSPDG environment instantiation,
    querying observations from the environment,
    sending actions to KSPDG environment,
    and cleanup of connections and environment at termination

    Args:
        env_cls : type[KSPDGBaseEnv]
            class of (not instance of) KSPDG environment to run agent within
        env_kwargs : dict
            keyword args to be passed when instantiating environment class
    """

    # create a separate logger becasue this is a separate process
    logger = create_logger(__name__, logging.DEBUG if debug else logging.INFO)

    def observation_handshake():
        # check for environment observation request from solver
        if observation_query_event.is_set():
            logger.debug("env observation request received. Transmitting...")
            obs_conn_send.send(env.get_observation())
            observation_query_event.clear()

    # Create environment
    logger.info("\n~~~Instantiating KSPDG envrionment~~~\n")
    env_cls = env_cls
    if env_kwargs is not None:
        env = env_cls(**env_kwargs, debug=debug)
    else:
        env = env_cls(debug=debug)
    _, env_info = env.reset()

    # execute accel schedule until agent termination
    agent_act = None
    while not termination_event.is_set():
        # check for environment observation request from solver
        observation_handshake()

        # wait for agent action
        if act_conn_recv.poll():
            logger.debug("receiving new agent action")
            agent_act = act_conn_recv.recv()

        # if no current action provided, loop back to start to wait for agent action
        if agent_act is None:
            continue

        # execute DG solution in KSP environment
        _, _, env_done, env_info = env.step(action=agent_act)

        # terminate loops if successful capture
        if env_done:
            logger.info("Environment Done")
            termination_event.set()
            break

        if termination_event.is_set():
            break

    # return procedure for mp.Process
    logger.info("\nsaving environment info...")
    return_dict["env_info"] = env_info

    # cleanup
    logger.info("\n~~~Closing KSPDG envrionment~~~\n")
    env.close()
