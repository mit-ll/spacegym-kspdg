# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

# Single-sourcing package version
# https://packaging.python.org/guides/single-sourcing-package-version/

__version__ = "0.14.0"

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
    

# import functions but name-mangle them so they are not 
# inadvertently top-level accessible in the kspdg package
import importlib
from importlib.util import find_spec
from kspdg.utils.private_src_utils import get_private_src_module_str as __get_mod_str


# import obfuscated evaluate.py
__evaluate_path = __get_mod_str("kspdg_envs.dist_evaluate")
try:
    evaluate = importlib.import_module(__evaluate_path)
except ModuleNotFoundError:
    print(f"Module {__evaluate_path} not found.")

# Condition import of LG3, LG4 environments on the presence of juliacall dependency
# Therefore, the kspdg library should be usable without the adv_bots optional 
# dependency
if find_spec('juliacall') is not None:
    # import obfuscated LBG1-LG3 environments
    __lg3_envs_path = __get_mod_str("kspdg_envs.lbg1.lg3_envs")
    try:
        __lg3_envs_module = importlib.import_module(__lg3_envs_path)
    except ModuleNotFoundError:
        print(f"Module {__lg3_envs_path} not found.")
    LBG1_LG3_I1_V1 = getattr(__lg3_envs_module, 'LBG1_LG3_I1_Env')
    LBG1_LG3_I2_V1 = getattr(__lg3_envs_module, 'LBG1_LG3_I2_Env')

    # import obfuscated LBG1-LG4 environments
    __lg4_envs_path = __get_mod_str("kspdg_envs.lbg1.lg4_envs")
    try:
        __lg4_envs_module = importlib.import_module(__lg4_envs_path)
    except ModuleNotFoundError:
        print(f"Module {__lg4_envs_path} not found.")
    LBG1_LG4_I1_V1 = getattr(__lg4_envs_module, 'LBG1_LG4_I1_Env')
    LBG1_LG4_I2_V1 = getattr(__lg4_envs_module, 'LBG1_LG4_I2_Env')

    # import obfuscated LBG1-LG5 environments
    __lg5_envs_path = __get_mod_str("kspdg_envs.lbg1.lg5_envs")
    try:
        __lg5_envs_module = importlib.import_module(__lg5_envs_path)
    except ModuleNotFoundError:
        print(f"Module {__lg5_envs_path} not found.")
    LBG1_LG5_I1_V1 = getattr(__lg5_envs_module, 'LBG1_LG5_I1_Env')
    LBG1_LG5_I2_V1 = getattr(__lg5_envs_module, 'LBG1_LG5_I2_Env')

    # import obfuscated LBG1-LG6 environments
    __lg6_envs_path = __get_mod_str("kspdg_envs.lbg1.lg6_envs")
    try:
        __lg6_envs_module = importlib.import_module(__lg6_envs_path)
    except ModuleNotFoundError:
        print(f"Module {__lg6_envs_path} not found.")
    LBG1_LG6_I1_V1 = getattr(__lg6_envs_module, 'LBG1_LG6_I1_Env')
    LBG1_LG6_I2_V1 = getattr(__lg6_envs_module, 'LBG1_LG6_I2_Env')

else:
    LBG1_LG3_I1_V1 = LBG1_LG3_I2_V1 = lambda *args, **kwargs: (
        "Unmet dependency juliacall for using LBG1_LG3 environments.\n" +
        "Please install kspdg[adv_bots] or kspdg[full] to use these.\n" +
        "Refere to README for further instructions."
    )

    LBG1_LG4_I1_V1 = LBG1_LG4_I2_V1 = lambda *args, **kwargs: (
        "Unmet dependency juliacall for using LBG1_LG4 environments.\n" +
        "Please install kspdg[adv_bots] or kspdg[full] to use these.\n" +
        "Refere to README for further instructions."
    )

    LBG1_LG5_I1_V1 = LBG1_LG5_I2_V1 = lambda *args, **kwargs: (
        "Unmet dependency juliacall for using LBG1_LG5 environments.\n" +
        "Please install kspdg[adv_bots] or kspdg[full] to use these.\n" +
        "Refere to README for further instructions."
    )

    LBG1_LG6_I1_V1 = LBG1_LG6_I2_V1 = lambda *args, **kwargs: (
        "Unmet dependency juliacall for using LBG1_LG6 environments.\n" +
        "Please install kspdg[adv_bots] or kspdg[full] to use these.\n" +
        "Refere to README for further instructions."
    )
