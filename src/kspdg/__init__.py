# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Single-sourcing package version
# https://packaging.python.org/guides/single-sourcing-package-version/

__version__ = "0.8.1-alpha.1"

# these imports make the individual environments accessible at the top-level
# of the library and assign an environment version number
# If the underlying environment changes, then the version number should be
# incremented

from kspdg.pe1.e1_envs import PE1_E1_I1_Env as PE1_E1_I1_V1
from kspdg.pe1.e1_envs import PE1_E1_I2_Env as PE1_E1_I2_V1
from kspdg.pe1.e1_envs import PE1_E1_I3_Env as PE1_E1_I3_V1
from kspdg.pe1.e1_envs import PE1_E1_I4_Env as PE1_E1_I4_V1
from kspdg.pe1.e2_envs import PE1_E2_I1_Env as PE1_E2_I1_V1
from kspdg.pe1.e2_envs import PE1_E2_I2_Env as PE1_E2_I2_V1
from kspdg.pe1.e2_envs import PE1_E2_I3_Env as PE1_E2_I3_V1
from kspdg.pe1.e2_envs import PE1_E2_I4_Env as PE1_E2_I4_V1
from kspdg.pe1.e3_envs import PE1_E3_I1_Env as PE1_E3_I1_V1
from kspdg.pe1.e3_envs import PE1_E3_I2_Env as PE1_E3_I2_V1
from kspdg.pe1.e3_envs import PE1_E3_I3_Env as PE1_E3_I3_V1
from kspdg.pe1.e3_envs import PE1_E3_I4_Env as PE1_E3_I4_V1
from kspdg.pe1.e4_envs import PE1_E4_I1_Env as PE1_E4_I1_V1
from kspdg.pe1.e4_envs import PE1_E4_I2_Env as PE1_E4_I2_V1
from kspdg.pe1.e4_envs import PE1_E4_I3_Env as PE1_E4_I3_V1
from kspdg.pe1.e4_envs import PE1_E4_I4_Env as PE1_E4_I4_V1

from kspdg.lbg1.lg0_envs import LBG1_LG0_I1_Env as LBG1_LG0_I1_V1
from kspdg.lbg1.lg0_envs import LBG1_LG0_I2_Env as LBG1_LG0_I2_V1
from kspdg.lbg1.lg1_envs import LBG1_LG1_I1_Env as LBG1_LG1_I1_V1
from kspdg.lbg1.lg1_envs import LBG1_LG1_I2_Env as LBG1_LG1_I2_V1
from kspdg.lbg1.lg2_envs import LBG1_LG2_I1_Env as LBG1_LG2_I1_V1
from kspdg.lbg1.lg2_envs import LBG1_LG2_I2_Env as LBG1_LG2_I2_V1

from kspdg.sb1.e1_envs import SB1_E1_I1_Env as SB1_E1_I1_V1
from kspdg.sb1.e1_envs import SB1_E1_I2_Env as SB1_E1_I2_V1
from kspdg.sb1.e1_envs import SB1_E1_I3_Env as SB1_E1_I3_V1
from kspdg.sb1.e1_envs import SB1_E1_I4_Env as SB1_E1_I4_V1
from kspdg.sb1.e1_envs import SB1_E1_I5_Env as SB1_E1_I5_V1

# Private-source, python-version-specific environments with advanced bots (e.g. julia-based)
import sys
if sys.version_info[:2] == (3, 12):
    # Python 3.12
    from kspdg.private_src.python3_12.kspdg_envs.lbg1.lg3_envs import LBG1_LG3_I1_Env as LBG1_LG3_I1_V1
    from kspdg.private_src.python3_12.kspdg_envs.lbg1.lg3_envs import LBG1_LG3_I2_Env as LBG1_LG3_I2_V1
elif sys.version_info[:2] == (3, 9):
    # Python 3.9
    from kspdg.private_src.python3_9.kspdg_envs.lbg1.lg3_envs import LBG1_LG3_I1_Env as LBG1_LG3_I1_V1
    from kspdg.private_src.python3_9.kspdg_envs.lbg1.lg3_envs import LBG1_LG3_I2_Env as LBG1_LG3_I2_V1
else:
    # Handle other versions or raise an error
    raise ImportError(f"Private-source environments require python 3.9 or 3.12, got {sys.version}")
