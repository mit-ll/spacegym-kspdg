# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import logging
import time

from kspdg.utils.loggers import create_logger

def ksp_interface_loop(
        env_cls, 
        env_kwargs, 
        obs_conn_send, 
        act_conn_recv, 
        termination_event, 
        observation_query_event, 
        return_dict,
        debug):
    """ Parallel process for running and interacting with environment
    
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
        termination_event : multiprocessing.Event
            flags all process to terminate (e.g. agent and environment)
        observation_query_event : multiprocessing.Event
            when set, runner is requesting an observation from environment
    """

    # create a separate logger becasue this is a separate process
    logger = create_logger(__name__, logging.DEBUG if debug else logging.INFO)

    def observation_handshake():
        # check for environment observation request from solver
        if observation_query_event.is_set():
            logger.debug("Env observation request received. Transmitting...")
            obs_conn_send.send(env.get_observation())
            observation_query_event.clear()

    # Create environment
    logger.info("\n~~~Instantiating KSPDG environment~~~\n")
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

        # execute agent's action in KSP environment
        if debug:
            env_step_start = time.time()
            logger.debug("Stepping environment...")
        _, _, env_done, env_info = env.step(action=agent_act)
        if debug:
            logger.debug(f"Environment step completed in {time.time()-env_step_start:.5g} sec")

        # terminate agent runner loops if environment flags a done episode
        if env_done:
            logger.info("Environment episode done. Terminating runner...")
            termination_event.set()
            break

        elif not env_done and termination_event.is_set():
            logger.info("Runner terminated before episode done")
            break

    # check to see if runner is waiting for an observation,
    # if so, send None so that runner can keep moving and 
    # identify the termination flag
    if observation_query_event.is_set():
        logger.info("Runner termination event has been set. Sending None observation...")
        obs_conn_send.send(None)

    # return procedure for mp.Process
    logger.info("saving environment info...")
    return_dict["agent_env_results"] =  env_info

    # cleanup
    logger.info("\n~~~Closing KSPDG environment~~~\n")
    env.close()