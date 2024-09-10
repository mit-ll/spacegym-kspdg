# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Functions that help with management and imports of obfuscated code ("private source" code)

# Private-source, python-version-specific, platform and architecture-specific
# environments with advanced bots (e.g. julia-based)
import sys
import platform

def get_python_version():
    # Get the Python version in the format 'python3_12'
    version_info = sys.version_info
    return f"python{version_info.major}_{version_info.minor}"

def get_supported_architecture():
    """Checks and returns architecture of user's machine while checking for compatibility"""

    # Define supported architectures
    supported_architectures = [
        'Darwin_x86_64',    # MacOS w/ Intel 
        'Darwin_arm64',     # MacOS w/ Apple Silicon
        'Windows_x86_64',    # Windows w/ Intel (confusingly Windows call this AMD64)
        'Linux_x86_64'      # Linux w/ Intel
    ]

    # Get the platform and architecture of user's machine
    platform_os = platform.system()
    machine = platform.machine()

    # Handle Windows architecture naming convention
    if platform_os == 'Windows' and machine == 'AMD64':
        # Rename architecture to align with pyarmor platform options to
        # handle Windows architecture naming convention
        # that uses x86_64 and AMD64 synonomously
        machine = 'x86_64'

    # compose architecture string and check for compatibility w/ 
    # obfuscated code from pyarmor
    architecture = f"{platform_os}_{machine}"
    if architecture not in supported_architectures:
        raise RuntimeError(f"Unsupported architecute: {architecture}")

    return architecture

def get_private_src_module_str(mod_name):
    """returns string of module location to obfuscated code based on python version and system architecture"""
    # Get dynamic parts of the import path
    python_version = get_python_version()
    platform_architecture = get_supported_architecture()

    # Build the module path
    return f"kspdg.private_src.{python_version}.{platform_architecture}.{mod_name}"