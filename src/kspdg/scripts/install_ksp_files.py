# Copyright (c) 2024, MASSACHUSETTS INSTITUTE OF TECHNOLOGY
# Subject to FAR 52.227-11 – Patent Rights – Ownership by the Contractor (May 2014).
# SPDX-License-Identifier: MIT

import os
import shutil
from pathlib import Path

from importlib.resources import files

KSPDG_INSTALL_PATH = files('kspdg')

def copy_ksp_files(ksp_files_path, ksp_game_path):
    """ recursively copy kspdg's mission files to ksp game installation, with replacement

    Args:
        ksp_files_path : str
            Path to kspdg libraries ksp_files directory
        ksp_game_path : str
            Path to KSP game installation
    """

    # map input 
    ksp_files_path = Path(ksp_files_path).expanduser().resolve()
    ksp_game_path = Path(ksp_game_path).expanduser().resolve()

    if not ksp_files_path.is_dir():
        raise FileNotFoundError(f"ksp_files directory does not exist: {ksp_files_path}")

    if not ksp_game_path.is_dir():
        raise FileNotFoundError(f"KSP game installation directory does not exist: {ksp_game_path}")
    
    # Recursively walk through the ksp_files source directory
    for src_file in ksp_files_path.rglob('*'):
        # Compute the corresponding destination file path
        relative_path = src_file.relative_to(ksp_files_path)
        dest_file = ksp_game_path / relative_path

        if src_file.is_dir():
            # Ensure destination subdirectory exists
            dest_file.mkdir(parents=True, exist_ok=True)
        elif src_file.is_file():
            # Copy file, overwriting only if it already exists
            shutil.copy2(src_file, dest_file)

def main():

    # Find kspdg's `ksp_files/` directory in kspdg install
    # ksp_files_path = os.path.join(KSPDG_INSTALL_PATH, "../../ksp_files")
    ksp_files_path = KSPDG_INSTALL_PATH / Path("../../ksp_files")
    ksp_files_path = ksp_files_path.resolve()

    # Query user for destination path
    ksp_game_path = input("Enter the path to your KSP game installation (e.g. ~/Desktop/KSP_osx/): ").strip()
    ksp_game_path = Path(ksp_game_path).expanduser().resolve()

    copy_ksp_files(ksp_files_path, ksp_game_path)

