# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

### Fixed

### Changed

- Conda environment is no longer pinned to python 3.9 due to removal of poliastro dependency

### Removed

- poliastro dependency and related functions in `utils.py`. This removes functions for propagating orbits which were used for non-essential metric evals. For further justification, see [this issue](https://github.com/mit-ll/spacegym-kspdg/issues/8)

## [v0.1.0] - 2023-10-23

### Added

- Single-value scoring function for pursuit-evade (pe1), sun-blocking (sb1), and lady-bandit-guard (lbg1) scenarios
- User id and passkey in evaluation process
- Agent instance name in agent_cfg for evaluation script
- Logs if episode is done in info
- Individual scenario environments are made accessible at top-level of package and version numbers added. e.g. `from kspdg import PE1_E1_I3_V1`. This also sets the beginning of version-marking environments. Future updates to almost any aspect of the environment (particularly observation, action, or reward) would imply the need for version increment
- New example baseline agents: random and passive

### Fixed

### Changed

- Upgrading kRPC dependency from v0.4.8 to v0.5.2 which fixed several issues that had required hard-pinning some dependencies (setuptools, protobuf). By un-pinning these, the library is more flexible / less brittle
- AgentEnvRunner timeout allowed to be None, will wait for environment episode to return done if runner timeout is None
- Expanding evaluation results string to log additional information about user, agent, environment, and kspdg version
- Evaluation results printed to a json-formatted txt file for ease of reading and unified uploading
- Improving termination of AgentEnvRunner so as not to wait for observation timeout on runner.py
- pe1 scenario reward function is now negative of scoring function and only assessed at termination (zero all other times)
- Renamed compiled evaluation script to evaluate.pyc
- Restructured where evaluation scripts, inputs (configs), and outputs (results) are housed in the repo

### Removed

## [v0.0.23] - 2023-09-07

### Added

- First implementation of sun-blocking environments (sb1) 
- `example_hello_kspdg.py` a "hello world" scirpt for defining an agent
- `agent_api` subpackage used for defining KSPDG solver agents that integrate and run within KSPDG environments in a streamlined, systematic fashion
- agent-environment evaluator with authentication tools. To be used for decentralized evalutation of user-defined agents

### Fixed

### Changed

- Improved, custom logger for environments (KSPDGBaseEnv) and agent-environment runners (BaseAgentEnvRunner)

### Removed

- Unused `scripts`: `lmp_safehandler.sh`, `lmp_safehandler_macos.sh`, `pf_lmp.conf` to clean up library

## [v0.0.22] - 2023-02-23