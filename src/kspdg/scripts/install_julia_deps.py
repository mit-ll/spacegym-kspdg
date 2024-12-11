# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# install necessary julia dependencies via juliacall and juliapkg

import os
import juliapkg     # this should have been installed during the juliacall install within conda env
from subprocess import CalledProcessError
from warnings import warn

def main():
    print("Installing Julia dependencies...")

    juliapkg.add("FromFile", "ff7dd447-1dcb-4ce3-b8ac-22a812192de7")
    # iLQGames.jl: Need URL since ilqgames does not appear to be registered
    juliapkg.add("iLQGames", "ae812560-bd7e-11e9-3bd3-b502f7cff3d3", url="https://github.com/lassepe/iLQGames.jl.git")

    # you should be able to see this change reflected in ~/miniconda3/envs/kspdg/julia_env/pyjuliapkg/juliapkg.json

    # resolves julia packages, ie ilqgames, may take a long time
    try:
        juliapkg.resolve()
    except CalledProcessError as e:
        # This error was seen on Windows machines due to configuration of SSL_CERT_FILE
        # workaround is to set an empty JULIA_SSL_CA_ROOTS_PATH env variable
        # and try to resolve again

        warn_color = '\033[93m'
        reset_color = '\033[0m'
        # colorized 
        warn(
            f"\n{warn_color}WARNING: Encountered error when resolving Julia environment (see above ERRORS). "
            f"Attempting workaround to use SSL_CERT_FILE...\n{reset_color}")
        
        if 'SSL_CERT_FILE' not in os.environ:
            raise NotImplementedError(
                "SSL_CERT_FILE not set in environment variable. " +
                f"No workaround yet implemented for CalledProcessError: {e}"
            )
        
        os.environ["JULIA_SSL_CA_ROOTS_PATH"] = ""
        juliapkg.resolve()