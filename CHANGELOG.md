# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v0.14.0] - 2025.12.08

### Added

- solve_lq_lbg1.jl: new julia module for linear-quadratic LBG game
- test_jl_dynamics.py: serverless tests for dynamics equations of motion written in julia
- test_jl_solvers.py: adding tests for solve_lq_lady_bandit_guard julia functions
- utils.py: added convert_rhntw_to_rhcbci, the inverse of convert_rhcbci_to_rhntw, and a common function between the two: compute_R_rhntw_rhcbci to compute the rotation matrix they both use
- private_src/: LBG1_LG6 environments and lbg1_private_utils
- lbg1_base.py: adding env state vars for lady and bandit capture to enable score modifiers upon capture
- run_tests.py and pyproject.toml: script for running aiaa competition tests

### Changed:

- plotters.py and runner.py: live telemetry viewer window will remain open for extended timeout (20sec), unless user manually closes, then it will forcefully close

## [v0.13.1] - 2025.10.21

### Added

- utils.py: add convert_rhcbci_to_rhntw for converting vectors from right-handed celestial-body-centered-inertial to right-handed NTW frame
- lbg1_base.py: new Hill-frame plot of bandit and guard positions relative to Lady

## [v0.13.0] - 2025.10.17

### Added

- pyproject.toml: added dearpygui dependency to new viz_tools optional dependency group
- utils/plotters.py: high-level, environment-agnostic code for creating and managing dearpygui plots (e.g. live telemetry)
- runner.py: real-time telemetry plots for visualizing scenario data (e.g. relative distances)
- lbg1_base.py, pe1_base.py: env-group-specific telemetry plotting functions dpg_setup and dpg_update

## [v0.12.3] - 2025.10.06

### Fixed

- add pyarmor .so and .pyd files that were neglected

## [v0.12.2] - 2025.10.03

### Added

- private_src/: python 3.10 support of obfuscated environments

## [v0.12.1] - 2025.09.09

### Changed

- Updating README.md to reflect changes in KSP purchasing instructions
- Updating obfuscated code to reflect kickoff of 2026 challenge

## [v0.12.0] - 2025.01.05

### Added

+ Obfuscated LBG1_LG5 challenge environments

### Changed 

+ Improving print functions in `install_ksp_files.py` called by `kspdg-install-ksp-files` entry point script
+ Improved default results/ directory handling/creation during kspdg-evaluate to make print commands less ambiguous about potential overwriting of existing directory

## [v0.11.1] - 2024.12.30

### Fixed

+ Typo in `kspdg-evaluate` entry point script
+ Instructions for installing from github to use `main` branch instead of `latest`

## [v0.11.0] - 2024.12.16

### Added

+ Explicit `pyyaml` dependency (I think this used to be a sub-dependency of astropy)
+ In-package `src/kspdg/scripts/` directory for holding command-line entry-point scripts (e.g. julia install, unit tests) so that full download/clone of kspdg is not needed for julia install and unit testing
+ `MANIFEST.in` to include tests in source distributions
+ Julia dependency entry point. Run with
```bash
kspdg-install-julia-deps
```

+ KSP files copy script entry point. This creates the GameData/KSPDG directory for easier management of evaluation configs and results and copies the KSPDG mission files into the KSP installation. Run with
```bash
kspdg-install-ksp-files
```

+ Serverless and in-game unit test entry points. Run with
```bash
kspdg-run-serverless-tests
kspdg-run-lbg1-i2-tests
kspdg-run-pe1-i3-tests
kspdg-run-sb1-i5-tests
```

+ evaluate.py command-line entry point. Run with
```bash
kspdg-evaluate path/to/cfg.yml optional/path/to/results/
```

### Changed

+ Moving `tests/`, `ksp_files/`, and `evaluation/` directories __into__ `src/kspdg/` so that they are considered package-data that is accessible at runtime by end users. This enables no-clone unit testing and ksp-file installation. For justification for avoiding use of non-package data, see: https://setuptools.pypa.io/en/latest/userguide/datafiles.html#non-package-data-files
+ Moving `install_julia_deps.py` into kspdg package and adding it as a command-line entry point within pyproject.toml to simplify install process
+ Updated readme with no-clone install instructions and added recommended graphics settings

### Removed

+ Unused `astropy` dependency and related instructions in readme
+ `environment.yml` and developer instructions in readme as they are not needed by, and may confuse, end-users and are easy to reprodocue for developers. For example, developers can use the following process:
```bash
# create conda development environment
conda create --name kspdg_dev python=3.12 ipython
conda activate kspdg_dev

# clone kspdg repo and install as editable
git clone git@github.com:mit-ll/spacegym-kspdg.git
cd spacegym-kspdg
pip install -e .[full]

# copy necessary game files to KSP install
kspdg-install-ksp-files

# instal juliaup and julia dependencies
curl -fsSL https://install.julialang.org | sh
kspdg-install-julia-deps
```

## [v0.10.0] - 2024.12.09

### Added

+ LBG1_LG4 private-source environments and unit-tests

### Fixed

+ Very basic check of bot_thread status to see if an error occurs; if so, an error is thrown in the MainThread (e.g. Group1BaseEnv) so that environment execution does not continue with the bot error unnoticed
+ Slight change in test timings to make test_lbg1_i2 results more reliable

## [v0.9.2] - 2024.12.03

### Fixed

+ Private-source julia files included as setuptools package-data so that they are present when pip installing kspdg
+ test_jl_solvers.py imports julia files from kspdg install point, not from relative path from test script, to ensure files are present at install point

## [v0.9.1] - 2024.12.02

### Added

+ Support for python 3.13 in private_src obfuscated code

## [v0.9.0] - 2024.09.10

### Added

+ Platform (e.g. Windows, Mac, Linux) and architecture (e.g. x86_64, arm64) specific private source environments and evaluate.py
+ New `utils/private_src_utils.py` module that provides helper functions for managing obfuscated code imports

### Fixed

+ Conditional statements for importing LBG1_LG3 environments so that the lighter-weight install of kspdg without `juliacall` dependency does not error on import
+ Handling of SSL_CERT_FILE in `install_julia_deps.py`

### Changed

+ Updating environment READMEs for accuracy
+ Updating top-level README to simplify installation process

### Removed

+ Removed unused astropy imports in `utils.py`

## [v0.8.1] - 2024.08.14

### Added

### Fixed

+ Handling of `.so` files in package build

### Changed

### Removed

## [v0.8.0] - 2024.08.13

### Added

+ `private_src/python3_XX/kspdg_envs/dist_evaluate.py` which replace
+ `evaluation/evaluate.py` that replaces `evaluation/evaluate.cpython-39.pyc` and `evaluation/evaluate.cpython-312.pyc` by automatically detecting which python version is in use and importing the appropriate `private_src/python3_XX/kspdg_envs/dist_evaluate.py` for use

### Fixed

### Changed

### Removed

- `evaluation/evaluate.cpython-39.pyc` and `evaluation/evaluate.cpython-312.pyc`, replaced with single `evaluation/evaluate.py` that in turn imports version-specific code from `private_src` 

## [v0.7.0] - 2024.08.13

### Added

- `private_src` directory to hold all compiled and/or obfuscated source code (e.g. to obfuscate bot policies in environments)
- 'lg3_envs.py` that use ilqgames-based Guard agent
- `juliacall` dependency including basic unit tests to check proper installation
- `iLQGames.jl` dependency and python script `install_julia_deps.py` for partially automating the install process
- new optional dependency groups in `pyproject.toml`: `adv_bots` with juliacall dependency and `full` which installs all other dependency groups
- `scripts/example_private_src_env_runner.py` as an example of running and debugging private-source, advanced-bot environment `LBG1_LG3_I2_V1`
- 'matplotlib' dependency for testing group
- 2025 AIAA SciTech competition anouncement to `README.md`

### Fixed

- Cleaning up inaccurate docstring

### Changed

- Updated `example_eval_cfg.yaml` and related instructions in README to point to the private-source LBG1_LG3_I2_V1 environment
- Expanded DEBUG logger for better introspection on thread execution

### Removed

- 'scripts/basic_evade_v20220509001.py' to cleanup unused code
- obsolete `pe20220516` environments


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
