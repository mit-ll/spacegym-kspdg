# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v0.6.2] - 2024.07.01

### Added

- `logger` to `KSPDGBaseAgent` for unified logging interface for all child class agents

### Fixed

### Changed

- `logger_name` for `KSPDGBaseEnv` to point to the child class environment name for better specificity of origin of log statements (i.e. a log statement with child's name but originating in the parent class is more traceable than a log statement with the parent's name but originating in the child class)

### Removed

## [v0.6.1] - 2024.05.22

### Added

- Ability to check versioning on `evaluation/evaluate.cpython-XXX.pyc`, which is different than kspdg version number. Check with
```bash
conda activate kspdg
python evaluation/evaluate.cpython-312.pyc --version
```

### Fixed

- Updated `evaluation/evaluate.cpython-XXX.pyc` to work with new kspdg packaging

### Changed

### Removed

## [v0.6.0] - 2024.05.15

### Added

- `pyproject.toml` for package and dependency management

### Fixed

### Changed

- Updated `setup.py` to delegate most package management to `pyproject.toml`; maintains the get_version functionality
- Updated copyright year in headers

### Removed

- `setup.py` and `version.py` since single-source version moved to `pyproject.toml` and `kspdg/__init__.py`. See https://packaging.python.org/en/latest/guides/single-sourcing-package-version/ 
- `requirements.txt` since there are no "pinned" or "concrete" dependencies as kspdg is not (yet?) intended to be released as a stand-alone app. For further information about the intended role of requirements.txt, see and [setup vs requirements](https://caremad.io/posts/2013/07/setup-vs-requirement/) and [use of requirements w/ pyproject](https://stackoverflow.com/questions/74508024/is-requirements-txt-still-needed-when-using-pyproject-toml)
- KSPDG Challenge announcement from README

## [v0.5.1] - 2024.01.09

_KSPDG Challenge @ SciTech 2024 - Finals Codebase_

### Added

- Printout of current score for audience engagement purposes

### Fixed

- infinite loop in lbg1_lg2 if proximity to lady never reached
- lbg1 smoketest overwrite

### Changed

- moved PARMAS.INFO.K_WEIGHTED_SCORE to parent environment class

### Removed

## [v0.5.0] - 2023.12.15

_KSPDG Challenge @ SciTech 2024 - Semi-Finals Codebase_

### Added

- New Lady-Bandit-Guard scenarios with active Lady spacecraft (LG2)

### Fixed

### Changed

- updating test file names and adding additional scenarios to the test files (e.g. `test_lbg1_lg0_i2.py` -> `test_lbg1_i2.py` so we can test lg1_i2 and lg2_i2 in the same file)
- Updating print statements to logger statements in pe1 e3_envs

### Removed

## [v0.4.4] - 2023-12-11

### Added

- [iss2](https://github.com/mit-ll/spacegym-kspdg/issues/2) Unit test to check for PhysicsRangeExtender (PRE) proper installation

### Fixed

- PRE installed instructions in README

### Changed

### Removed

## [v0.4.3] - 2023-11-29

### Added

### Fixed

- bug where PARAMS from LBG1 classes overwrite PARAMS in PE1 classes due to top-level imports at kspdg/__init__.py.

### Changed

### Removed

## [v0.4.2] - 2023-11-28

### Added

### Fixed

### Changed

- Updated which tests to be run, i.e. `sb1_e1_i5`

### Removed

- 20220516_PursuitEvade mission files
- Tests for 20220629 pursuit scenario

## [v0.4.1] - 2023-11-22

### Added

### Fixed

- Typo in SB1_E1_I5_V1

### Changed

### Removed

## [v0.4.0] - 2023-11-19

### Added

- New field in composite action space "vec_type" which is a discrete variable that can be used to indicate if the action vector represents throttle values (-1, 1 in each axes) or thrust values (in Newtons in each axes)

### Fixed

### Changed

- further abstracted code from `pe1_base.PursuitEvadeGroup1Env` and `lbg1_base.LadyBanditGuardGroup1Env` into `base_envs.Group1BaseEnv` which defines the composite action space shared across the child classes

### Removed

## [v0.3.0] - 2023-11-13

### Added

- New composite action space for `pe1` environments that allows you to define the reference frame in which the burn is to be interpretted. The composite action space uses dictionaries as actions with two fields "burn_vec" and "ref_frame". burn_vec is the same format as the old action space (Box action space). ref_frame is an integer (Discrete action space). Backward compatibility with old action space has been maintained. If a list-like object is passed in, it will treat it as the old action space, otherwise a dict with appropriate fields must be passed
- Addtional `test_pe1_e1_i3.py` tests for reference frames
- Example agent using new action space and NTW reference frame. See `ProgradePursuitAgent` in `agent_api/example_agent.py`

### Fixed

### Changed

- Abstracted `convert_rhntw_to_rhpbody` and `convert_rhcbci_to_rhpbody` to parent level functions requiring as input the vessel object for which conversion to right-hand body coords is to be performed
- Abstracted `_start_bot_threads()` and `close()` to KSPDGBaseEnv to reduce redundancy
- Abstracted the `step` function from `pe1`, `lbg1`, and `sb1` environments up to KSPDGBaseEnv under the `step_v1` function

### Removed

- Commented code in lbg1_base.py, `observation_dict_to_list` and `observation_list_to_dict`

## [v0.2.0] - 2023-11-05

### Added

### Fixed

- Resolve [iss7](https://github.com/mit-ll/spacegym-kspdg/issues/7) by switching print statements to logger statements in lbg1_base.py

### Changed

- Conda environment is no longer pinned to python 3.9 due to removal of poliastro dependency. This will cause the most recent python version (currently 3.12) to be used when creating the environment
- `evaluate.pyc` script has been replaced by python-version-specific binaries `evaluate.cpython-39.pyc` (for python 3.9) and `evaluate.cpython-312.pyc` (for python 3.12). This is to avoid a `RuntimeError: Bad magic number in .pyc file` when trying to run evaluate.pyc from the wrong python version

### Removed

- poliastro dependency and related functions in `utils.py`. This removes functions for propagating orbits which were used for non-essential metric evals. For further justification, see [this issue](https://github.com/mit-ll/spacegym-kspdg/issues/8)
- `utils.estimate_capture_dv` and related unit tests which depended upon poliastro
- `utils.solve_lambert` and related unit tests which depended upon poliastro
- `utils.propagate_orbit_tof` and related unit tests which depended upon poliastro

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