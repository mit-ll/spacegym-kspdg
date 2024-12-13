# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# calls private-source evaluate.py based upon system python version

# import sys

# if sys.version_info[:2] == (3, 12):
#     # Python 3.12
#     from kspdg.private_src.python3_12.kspdg_envs.dist_evaluate import main
# elif sys.version_info[:2] == (3, 9):
#     # Python 3.9
#     from kspdg.private_src.python3_9.kspdg_envs.dist_evaluate import main
# else:
#     # Handle other versions or raise an error
#     raise ImportError(f"evaluate.py requires python 3.9 or 3.12, got {sys.version}")

import kspdg

def run_evaluation():
    kspdg.evaluate.main()
    
if __name__ == "__main__":
    run_evaluation()
