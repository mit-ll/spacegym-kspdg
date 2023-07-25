# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- First implementation of sun-blocking environments (sb1) 
- `newbie_pursuer.py` a "hello world" scirpt for defining an agent
- `agent_api` subpackage used for defining KSPDG solver agents that integrate and run within KSPDG environments in a streamlined, systematic fashion

### Fixed

### Changed

- Improved, custom logger for environments (KSPDGBaseEnv) and agent-environment runners (BaseAgentEnvRunner)

### Removed

- Unused `scripts`: `lmp_safehandler.sh`, `lmp_safehandler_macos.sh`, `pf_lmp.conf` to clean up library

## [v0.0.22] - 2023-02-23