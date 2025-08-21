#!/usr/bin/env python3
"""
Tools to process defaults and scripts for a SITL frame.
"""
import os
import json
import shutil
import functools
from typing import Any
from pathlib import Path

import ac_config_tools
from paths import CXPILOT_CONFIG_ROOT

SITL_FRAMES_JSON = CXPILOT_CONFIG_ROOT / "sitl" / "sitl_frames.json"


@functools.lru_cache(maxsize=1)
def get_frames() -> dict[str, dict[str, Any]]:
    """
    Extract SITL frame information from the sitl_frames.json file.
    Returns:
        dict: Dictionary containing frame information, where keys are frame names
              and values are dictionaries with frame details.
    """
    if not SITL_FRAMES_JSON.exists():
        raise FileNotFoundError(f"SITL frames JSON file not found: {SITL_FRAMES_JSON}")
    return json.loads(SITL_FRAMES_JSON.read_text(encoding='utf-8'))


def copy_scripts(frame_name: str, dest_root: Path, symlink: bool = False) -> None:
    """
    Copy scripts for a SITL frame to the specified destination folder.

    Args:
        frame_info (dict): Frame information dictionary.
        dest_root (str): Destination directory to copy scripts to.
        symlink (bool): If True, use symlinks instead of copying files.
    """
    frame_info = get_frames()[frame_name]
    base_config = frame_info.get("base_aircraft_config", "")
    if base_config:
        ac_config_tools._copy_scripts(base_config, dest_root, symlink=symlink)

    # Tuple of (abs_source_path, abs_destination_path), one for each script
    lua_scripts = []  # type: list[tuple[Path, Path]]
    # Now, get the scripts from the frame info itself. This is a list of glob
    # patterns to match the scripts to copy. Each pattern can be a string
    # or a list of two strings. If a list, the first string is the pattern to
    # match, and the second string is the destination folder or destination
    # file (to rename the script). Renaming is only allowed for a single file
    # (no wildcards in the pattern). If a single string, the script will be
    # copied to the root of the destination folder.
    script_patterns = frame_info.get("scripts", [])
    for pattern in script_patterns:
        # Determine the destination path
        if isinstance(pattern, list) and len(pattern) == 2:
            script_dest = Path(pattern[1])
            # Disallow wildcards in the destination path
            if '*' in script_dest.name or '?' in script_dest.name:
                raise ValueError(f"{frame_name}: script destination cannot contain wildcards: {script_dest}")
            pattern = pattern[0]
            script_dest = dest_root / script_dest
        elif isinstance(pattern, str):
            script_dest = dest_root
        else:
            raise ValueError(
                f"{frame_name}: script pattern must be a string or a list of two strings: {pattern}"
            )
        script_dest = script_dest.resolve()

        # If script_dest ends in .lua, then we are renaming a single file
        if script_dest.suffix == '.lua':
            file = CXPILOT_CONFIG_ROOT / pattern
            if not file.exists():
                raise FileNotFoundError(f"{frame_name}: script file not found: {file}")
            lua_scripts.append((file, script_dest))
        else:
            # Expand the pattern
            for file in (CXPILOT_CONFIG_ROOT).glob(pattern):
                lua_scripts.append((
                    file,
                    script_dest / file.name
                ))

        # Now copy the scripts to the destination folder
        for src, dst in lua_scripts:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if symlink:
                if dst.is_symlink() or dst.exists():
                    dst.unlink()
                dst.symlink_to(src)
            else:
                shutil.copy(src, dst)


def write_defaults_file(frame_name: str, defaults_out: Path, strip: bool = False) -> None:
    """
    Write the processed defaults file for a SITL frame.

    Args:
        frame_name (str): Name of the SITL frame to process.
        defaults_out (str): Path to write the processed defaults file to.
        strip (bool): If True, strip comments and whitespace from the defaults file.
    """
    frame_info = get_frames()[frame_name]
    defaults_file_path = CXPILOT_CONFIG_ROOT / frame_info.get("defaults", "")
    processed_defaults = ac_config_tools._process_defaults(defaults_file_path)
    if strip:
        processed_defaults = ac_config_tools._strip_defaults(processed_defaults)
    defaults_out.parent.mkdir(parents=True, exist_ok=True)
    defaults_out.write_text("\n".join(processed_defaults) + "\n", encoding='utf-8')


def get_model(frame_info: dict) -> tuple[str, Path | None]:
    """
    Get the -M argument to pass to the SITL command for this frame. This
    also handles adding the ip address for "flightaxis" models if needed.

    Args:
        frame_info (dict): Frame information dictionary.
    Returns:
        tuple: A tuple containing the model string and the path to the model
               JSON file if applicable
    """
    model = frame_info.get("model", "")
    if model.startswith("flightaxis"):
        realflight_ip = os.environ.get("REALFLIGHT_IP", "") or os.environ.get("wslhost", "")
        if realflight_ip:
            model = f"flightaxis:{realflight_ip.strip()}"
    if ':' in model and model.endswith('.json'):
        base_model, model_json = model.split(':', 1)
        model_json_path = CXPILOT_CONFIG_ROOT / model_json.strip()
        if not model_json_path.exists():
            raise FileNotFoundError(f"Model JSON file not found: {model_json_path}")
        return base_model.strip(), model_json_path.resolve()
    return (model, None)
