# Copyright (c) 2023, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# This script evaluates a user-defined agent within a user-specified environment

import sys
import argparse
import importlib
import yaml

from pathlib import Path

from kspdg.agent_api.runner import AgentEnvRunner
from kspdg.pe1.e1_envs import PE1_E1_I3_Env

def instantiate_agent(agent_module_path, agent_cls_name, agent_kwargs):
    """
    Dynamically imports an agent class from a Python module and instantiates an agent.

    Args:
        agent_module_path (str): The relative or absolute path to the Python module containing the agent class.
        agent_cls_name (str): The name of the agent class to be instantiated.
        agent_kwargs (dict): Additional keyword arguments required to instantiate the agent. These arguments
                  will be passed to the agent class's constructor.

    Returns:
        obj: An instance of the agent class initialized with the provided keyword arguments.


    Example:
        Assuming 'your_agent_module.py' contains a class named 'YourAgentClass':

        >>> agent_module_path = "path.to.your_agent_module"
        >>> agent_cls = "YourAgentClass"
        >>> agent_kwargs = {"arg1": value1, "arg2": value2}
        >>> agent_instance = instantiate_agent(agent_module_path, agent_cls, **agent_kwargs)

        The 'agent_instance' variable will now hold an instance of 'YourAgentClass'
        with the provided keyword arguments.
    """

    # Dynamically import the agent module
    agent_module_name = Path(agent_module_path).stem
    agent_module_spec = importlib.util.spec_from_file_location(agent_module_name, agent_module_path)
    agent_module = importlib.util.module_from_spec(agent_module_spec)
    sys.modules[agent_module_name] = agent_module
    agent_module_spec.loader.exec_module(agent_module)

    # Get the agent class from the module
    agent_class = getattr(agent_module, agent_cls_name)

    # Instantiate the agent with provided keyword arguments
    agent_instance = agent_class(**agent_kwargs)
    return agent_instance

def main():
    # Create a command-line argument parser
    parser = argparse.ArgumentParser(description="Instantiate an agent from a YAML configuration file.")

    # Add arguments
    parser.add_argument("agent_cfg", type=str, help="Path to the YAML configuration file for agent object.")
    parser.add_argument("env_mod", type=str, help="Module that holds enviornment class definition, relative to kspdg package")
    parser.add_argument("env_cls", type=str, help="Environment class in which agent will be evaluated.")

    # Parse the command-line arguments
    args = parser.parse_args()

    # Load the agent configuration from the YAML file
    with open(args.agent_cfg, "r") as config_file:
        agent_config = yaml.safe_load(config_file)

    # Extract the necessary fields from the configuration
    agent_module_path = agent_config["agent_module_path"]
    agent_cls = agent_config["agent_cls"]
    agent_kwargs = agent_config.get("kwargs", {})

    # Instantiate the agent
    agent = instantiate_agent(agent_module_path, agent_cls, agent_kwargs)

    # # convert environment class from str to cls
    # env_cls = globals()[args.env_cls]
    
    # attempt to import envrionment class
    env_module = importlib.import_module("kspdg."+args.env_mod)
    env_cls = getattr(env_module, args.env_cls)

    # create agent runner and run
    runner = AgentEnvRunner(agent, env_cls, env_kwargs=None, runner_timeout=30, debug=True)
    eval_results = runner.run()
    print(eval_results)

    # hash the results with key
    

if __name__ == "__main__":
    main()


