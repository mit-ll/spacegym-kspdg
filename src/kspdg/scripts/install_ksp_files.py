# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import shutil

from copy import copy
from pathlib import Path
from importlib.resources import files

KSPDG_INSTALL_PATH = files('kspdg')

def copy_ksp_mission_files(kspdg_path_in, ksp_game_path_in):
    """ recursively copy kspdg's mission files to ksp game installation, with replacement

    Args:
        kspdg_path_in : str
            Path to kspdg installation
        ksp_game_path_in : str
            Path to KSP game installation
    """

    # Convert inputs to Path objects and resolve user and relative paths
    kspdg_path = Path(kspdg_path_in).expanduser().resolve()
    ksp_game_path = Path(ksp_game_path_in).expanduser().resolve()

    # Define the expect path to ksp_files in the kspdg install
    kspdg_ksp_files_path = kspdg_path / "ksp_files"

    # Check for existence of all relavant paths
    if not kspdg_path.is_dir():
        raise FileNotFoundError(f"kspdg install directory does not exist: {kspdg_path}")
    if not ksp_game_path.is_dir():
        raise FileNotFoundError(f"KSP game installation directory does not exist: {ksp_game_path}")
    if not kspdg_ksp_files_path.is_dir():
        raise FileNotFoundError(f"ksp_files directory does not exist in kspdg install: {kspdg_path}")
    
    # Recursively walk through the ksp_files source directory
    print(f"Recursively copying and overwriting KSPDG mission files from '{kspdg_ksp_files_path}' to `{ksp_game_path}`.")
    for src_file in kspdg_ksp_files_path.rglob('*'):
        # Compute the corresponding destination file path
        relative_path = src_file.relative_to(kspdg_ksp_files_path)
        dest_file = ksp_game_path / relative_path

        if src_file.is_dir():
            # Ensure destination subdirectory exists
            dest_file.mkdir(parents=True, exist_ok=True)
            # print(f"Created directory '{dest_file}', NOT overwriting if already exists.")
        elif src_file.is_file():
            # Copy file, overwriting if it already exists
            shutil.copy2(src_file, dest_file)
            # print(f"Copied from '{src_file}' to '{dest_file}', overwriting if already exists.")

def setup_kspdg_game_data_dir(kspdg_path_in, ksp_game_path_in):
    """
    Sets up GameData/KSPDG/ within the KSP installation for better management of eval configs and results

    Args:
        kspdg_path_in : str
            Path to kspdg installation
        ksp_game_path_in : str
            Path to KSP game installation
    """
    # Convert inputs to Path objects and resolve user and relative paths
    kspdg_path = Path(kspdg_path_in).expanduser().resolve()
    ksp_game_path = Path(ksp_game_path_in).expanduser().resolve()

    # Define the expected path to pre-existing subdirectories
    kspdg_eval_path = kspdg_path / "evaluation"
    ksp_gamedata_path = ksp_game_path / "GameData"

    # Check for existence of all relavant paths
    if not kspdg_path.is_dir():
        raise FileNotFoundError(f"kspdg install directory does not exist: {kspdg_path}")
    if not ksp_game_path.is_dir():
        raise FileNotFoundError(f"KSP game installation directory does not exist: {ksp_game_path}")
    if not kspdg_eval_path.is_dir():
        raise FileNotFoundError(f"ksp_files directory does not exist in kspdg install: {kspdg_path}")
    
    # Check if GameData exists in KSP
    if not ksp_gamedata_path.is_dir():
        raise FileNotFoundError(f"GameData directory does not exist in KSP install path: {ksp_game_path}. Ensure you have provided valid install path of KSP Game")
    
    # Step 2: Check if KSPDG exists in GameData, and create it if it doesn't exist
    ksp_kspdg_path = ksp_gamedata_path / "KSPDG"
    if not ksp_kspdg_path.exists():
        ksp_kspdg_path.mkdir(mode=0o755, parents=True)
        print(f"Created directory: {ksp_kspdg_path}")
    else:
        print(f"Directory already exists: {ksp_kspdg_path}")
    
    # Step 3: Check for "configs" and "results" in "KSPDG", create if they don't exist
    for subdirectory in ["configs", "results"]:
        subdirectory_path = ksp_kspdg_path / subdirectory
        if not subdirectory_path.exists():
            subdirectory_path.mkdir(mode=0o755)
            print(f"Created directory: {subdirectory_path}")
        else:
            print(f"Directory already exists: {subdirectory_path}")
    
    # Step 4: Check if "configs/example_eval_cfg.yaml" exists; copy from kspdg install if it doesn't
    ksp_example_eval_cfg_path = ksp_kspdg_path / "configs" / "example_eval_cfg.yaml"
    kspdg_example_eval_cfg_path = kspdg_eval_path / "configs" / "example_eval_cfg.yaml"
    if not ksp_example_eval_cfg_path.exists():
        if not kspdg_example_eval_cfg_path.is_file():
            raise FileNotFoundError(f"Example eval config not found in kspdg install: {kspdg_example_eval_cfg_path}")
        
        shutil.copy(kspdg_example_eval_cfg_path, ksp_example_eval_cfg_path)
        print(f"Copied example_eval_cfg.yamlfrom '{kspdg_example_eval_cfg_path}' to '{ksp_example_eval_cfg_path}'.")
    else:
        print(f"File 'example_eval_cfg.yaml' already exists in '{ksp_example_eval_cfg_path}'. It will not be overwritten.")

def main():

    # Query user for destination path
    ksp_game_path = input("Enter the path to your KSP game installation (e.g. ~/Desktop/KSP_osx/): ").strip()
    ksp_game_path = Path(ksp_game_path).expanduser().resolve()

    # setup KSPDG directory in KSP's GameData
    setup_kspdg_game_data_dir(KSPDG_INSTALL_PATH, ksp_game_path)

    # copy mission files to KSP's saves and Missions directories
    copy_ksp_mission_files(KSPDG_INSTALL_PATH, ksp_game_path)


