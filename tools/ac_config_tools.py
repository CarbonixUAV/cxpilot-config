#!/usr/bin/env python3
"""
Command-line tool for reading aircraft config files and producing build inputs.

Used by the build scripts to keep them schema-agnostic. All schema-knowledge
and validation lives here (placeholders, defaults, Lua scripts, extra hwdef
composition, base CPN mapping).
"""

import re
import shutil
import fnmatch
import functools
import subprocess
from pathlib import Path
from typing import Optional, Iterable, Union
import xml.etree.ElementTree as ET
from datetime import datetime, timezone

from paths import CXPILOT_ROOT, CXPILOT_CONFIG_ROOT, CXPILOT_CORE_ROOT

AC_CONFIG_ROOT = CXPILOT_CONFIG_ROOT / "aircraft_configuration"

# We build AP_Periph with a long random string as the board name. This random
# string is replaced with the actual board name in the final binary using
# cx_apj_tool (an enhanced version of ArduPilot's apj_tool). It is chosen to be
# long enough to accommodate the longest board name that we expect to use.
# Currently, we use 40, which is a little larger than the longest board name we
# have used so far.
MAGIC_BOARD_NAME = "lm3eBX7cJeaUer67lXdkNr83q2WzPRbE2MxAgnGq"


@functools.lru_cache(maxsize=None)
def get_fc_board_name(config: str) -> str:
    """
    Get the flight controller board name for a given aircraft configuration.

    Args:
        config (str): Name of the aircraft configuration.

    Returns:
        str: The flight controller board name.
    """
    xml_file = _get_config_index()[config]["xml_file"]
    root = ET.parse(xml_file).getroot()
    flight_controller = root.find('flight_controller')
    if flight_controller is None:
        raise ValueError(f"'flight_controller' element not found in {xml_file}")
    board_name = flight_controller.find('board_name')
    if board_name is None or board_name.text is None:
        raise ValueError(f"'board_name' element not found in {xml_file}")
    return board_name.text.strip()


def generate_cpn_firmware(config: str, base_dir: Path, out_dir: Path) -> None:
    """
    Generate final CPN firmware files for the specified configuration.

    Args:
        config (str): Name of the aircraft configuration.
        base_dir (str): Path to the directory containing periph base firmware files.
        out_dir (str): Output directory for the generated CPN firmware files.
    """
    import cx_apj_tool
    if config not in _get_config_index():
        raise ValueError(f"Unknown configuration: {config}")

    # Raise an error if the destination directory isn't empty
    if out_dir.exists() and any(out_dir.iterdir()):
        raise FileExistsError(f"Output directory {out_dir} is not empty. Please clear it before running this command.")
    out_dir.mkdir(parents=True, exist_ok=True)

    board_names_set = set()
    for cpn in _get_managed_cpns(config):
        board_name = cpn.find('board_name')
        if board_name is None or board_name.text is None:
            raise ValueError(
                f"'board_name' element not found in CPN element for {config}: {ET.tostring(cpn, encoding='unicode')}"
            )
        board_name = board_name.text.strip()
        if board_name in board_names_set:
            raise ValueError(f"Duplicate board name found: {board_name} in CPN element for {config}")
        board_names_set.add(board_name)

        print(f"Generating CPN firmware: {board_name}")

        # Get the defaults file for this CPN
        defaults_file = cpn.find('waf_defaults')
        if defaults_file is None or defaults_file.text is None:
            raise ValueError(
                f"'waf_defaults' element not found in CPN element for {config}: {ET.tostring(cpn, encoding='unicode')}"
            )
        else:
            defaults_file = CXPILOT_CONFIG_ROOT / defaults_file.text.strip()
            # Process the defaults file
            processed_defaults = _process_defaults(defaults_file)
            # Always strip defaults for CPNs
            processed_defaults = _strip_defaults(processed_defaults)
            if not processed_defaults:
                raise ValueError(f"CPN defaults file for {board_name} is empty after processing")
            defaults_file = out_dir / f"{board_name}.defaults.parm"
            defaults_file.write_text('\n'.join(processed_defaults) + '\n')

        base_firmware_src = _get_cpn_base_firmware_name(cpn)
        base_firmware_src = base_dir / f"{base_firmware_src}.bin"
        if not base_firmware_src.exists():
            raise FileNotFoundError(f"Base firmware file {base_firmware_src} does not exist for {board_name}")

        # Use cx_apj_tool to generate the final firmware
        output_file = cx_apj_tool.embedded_defaults(str(base_firmware_src))
        output_file.filename = f"{out_dir / board_name}.bin"
        # Embed the defaults file into the firmware
        if not output_file.find():
            raise LookupError(f"Could not find a default param section to embed in {base_firmware_src}")
        output_file.set_file(defaults_file)
        output_file.firmware = cx_apj_tool.replace_board_name(output_file.firmware, MAGIC_BOARD_NAME, board_name)
        output_file.firmware = cx_apj_tool.fix_app_descriptor(output_file.firmware)

        output_file.save()


def get_periph_bases(configs: Optional[Iterable[str]] = None, allow_deprecated: bool = False) -> list[str]:
    """
    Get all the unique periph firmware files that waf will need to build before
    the final CPN firmware files (with modified board names and defaults) can
    be generated using apj-tool.

    CPN build CI would take forever if we just rebuild the dozens of unique
    final binaries for each CPN. We use this to bring the number of waf builds
    down to about three. Then we come back and modify the built firmware
    with the correct board name and default parameters.

    This returns a set of base firmware names. Any of these can be passed to
    `periph_base_prepare` to generate the extra_hwdef.dat file for that base.
    """
    config_index = _get_config_index()
    if configs is None:
        configs = list(config_index.keys())
        # Only filter out deprecated configs if they weren't explicitly requested
        if not allow_deprecated:
            configs = [c for c in configs if config_index[c]["status"] == "active"]

    # Make sure configs are valid
    for config in configs:
        if config not in config_index:
            raise ValueError(f"Unknown configuration: {config}")

    periph_bases = _get_periph_bases_all()
    periph_bases_filtered = set()
    for base_name, info in periph_bases.items():
        if info["configs"].intersection(configs):
            periph_bases_filtered.add(base_name)
    return sorted(periph_bases_filtered)


def get_configs(allow_deprecated: bool = False) -> list[str]:
    """
    Get a list of all available aircraft configurations.

    Returns:
        list[str]: List of configuration names.
    """
    config_index = _get_config_index()
    configs = sorted(config_index.keys())
    if not allow_deprecated:
        configs = [c for c in configs if config_index[c]["status"] == "active"]
    return configs


def get_flight_controller_env() -> dict:
    """
    Set environment variables for the embedding the git hash in the flight
    controller firmware during the build.
    """
    # We embed the integration commit ID as the ArduPilot hash, since this
    # alone (if clean; the case for all official builds) is enough to know the
    # state of all three repositories.
    integration_id, _, _ = _get_git_commit_ids()
    return _get_git_version_env_vars(integration_id)


def prepare_flight_controller(
    config: str,
    version: str,
    xml_out: Path,
    scripts_out: Path,
    defaults_out: Path,
    extra_hwdef_out: Path,
    strip_defaults: bool = False,
    symlink_scripts: bool = False,
) -> None:
    """
    Prepare the flight controller configuration for building.

    This function processes the XML configuration, writes the processed XML,
    copies the Lua scripts, writes the flight controller defaults, and generates
    the extra hardware definition file.

    Args:
        config (str): Name of the aircraft configuration.
        version (str): Version string to embed in the firmware.
        xml_out (str): Path to write the processed XML file to.
        scripts_out (str): Path to copy the Lua scripts to.
        defaults_out (str): Path to write the flight controller defaults file to.
        extra_hwdef_out (str): Path to write the extra hardware definition file to.
        strip_defaults (bool): If True, strip comments and whitespace from the defaults file.
        symlink_scripts (bool): If True, use symlinks for the Lua scripts instead
                                of copying them.
    """
    _write_processed_xml(
        config=config,
        xml_out=xml_out,
        version=version,
    )
    _copy_scripts(
        config=config,
        dest_root=scripts_out,
        symlink=symlink_scripts,
    )
    _write_fc_defaults(
        config=config,
        defaults_out=defaults_out,
        strip=strip_defaults,
    )
    _write_fc_extra_hwdef(
        config=config, custom_version=version, out_path=extra_hwdef_out
    )


def prepare_base_periph_firmware(base_name: str, version: str, extra_hwdef_out: Path) -> None:
    """
    Prepare the base periph firmware for building.

    This function retrieves the base firmware information, writes the extra hardware definition file,
    and returns the board name associated with the base firmware.
    """
    bases_all = _get_periph_bases_all()
    if base_name not in bases_all:
        raise ValueError(f"Unknown periph base: {base_name}")
    base_info = bases_all[base_name]
    files = base_info["waf_extra_hwdef"]
    _write_periph_extra_hwdef(files, version, extra_hwdef_out)


def get_waf_board(base_name: str) -> str:
    """
    Get the waf board name for a given base firmware name.

    Args:
        base_name (str): The base firmware name.

    Returns:
        str: The board name to pass to waf configure
    """
    bases_all = _get_periph_bases_all()
    if base_name not in bases_all:
        raise ValueError(f"Unknown periph base: {base_name}")
    return bases_all[base_name]["waf_board"]


@functools.lru_cache(maxsize=1)
def _get_config_index() -> dict[str, dict[str, Union[Path, str]]]:
    """
    Get a dictionary index of all aircraft configurations.

    The keys are the configuration names, and the values are the paths to the
    XML files defining the configurations.
    """
    index = {}
    for xml_file in sorted(AC_CONFIG_ROOT.glob('*.xml')):
        try:
            root = ET.parse(xml_file).getroot()
        except ET.ParseError as e:
            raise RuntimeError(f"Failed to parse XML file {xml_file}: {e}")
        aircraft = root.find('aircraft')
        if aircraft is None:
            raise ValueError(f"'aircraft' element not found in {xml_file}")
        model = aircraft.find('model')
        if model is None or model.text is None:
            raise ValueError(f"'model' element not found in the 'aircraft' element of {xml_file}")
        model_version = aircraft.find('model_version')
        if model_version is None or model_version.text is None:
            raise ValueError(f"'model_version' element not found in the 'aircraft' element of {xml_file}")
        config_name = f"{model.text.strip()}_{model_version.text.strip()}"
        if config_name in index:
            raise RuntimeError(
                f"Duplicate config name found: {config_name} in {xml_file} and {index[config_name]['xml_file']}"
            )
        status = aircraft.find('status')
        if status is None or status.text is None:
            raise ValueError(f"'status' element not found in the 'aircraft' element of {xml_file}")
        if status.text.strip() not in ("active", "deprecated"):
            raise ValueError(f"Invalid status '{status.text.strip()}' in {xml_file}. Must be 'active' or 'deprecated'.")
        index[config_name] = {
            "xml_file": xml_file,
            "status": status.text.strip(),
        }

    return index


def _write_processed_xml(config: str, xml_out: Path, version: str) -> None:
    """
    Write the processed XML file for a given aircraft configuration.

    Args:
        config (str): Name of the aircraft configuration.
        xml_out (str): Path to write the processed XML file to.
    """
    xml_file = Path(_get_config_index()[config]["xml_file"])
    integration_id, core_id, config_id = _get_git_commit_ids()
    # Replace placeholders in the XML content
    content = xml_file.read_text()
    content = content.replace('$cx_pilot_version', version)
    content = content.replace('$cx_pilot_build_date', datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'))
    content = content.replace('$cx_pilot_commit_id', integration_id)
    content = content.replace('$cx_pilot_core_commit_id', core_id)
    content = content.replace('$cx_pilot_config_commit_id', config_id)

    # Make sure we processed all placeholders, and that no new ones were introduced that we didn't handle
    leftover = sorted(set(m.group(0) for m in re.finditer(r'\$cx_pilot[A-Za-z0-9_]*', content)))
    if leftover:
        raise RuntimeError(f"{xml_file} unknown placeholders found: {', '.join(leftover)}")

    # Write the processed content
    xml_out.parent.mkdir(parents=True, exist_ok=True)
    xml_out.write_text(content)


def _write_fc_defaults(config: str, defaults_out: Path, strip: bool = False) -> None:
    """
    Write the processed defaults file for the flight controller.

    Args:
        config (str): Name of the aircraft configuration.
        defaults_out (str): Path to write the processed defaults file to.
        strip (bool): If True, strip comments and whitespace from the defaults file.
    """
    xml_file = _get_config_index()[config]["xml_file"]
    root = ET.parse(xml_file).getroot()
    flight_controller = root.find('flight_controller')
    if flight_controller is None:
        raise ValueError(f"'flight_controller' element not found in {xml_file}")
    defaults_file = flight_controller.find('defaults_file')
    if defaults_file is None or defaults_file.text is None:
        raise ValueError(f"'defaults_file' element not found in {xml_file}")

    # defaults_file_path = os.path.join(CXPILOT_CONFIG_ROOT, defaults_file.text)
    defaults_file_path = CXPILOT_CONFIG_ROOT / defaults_file.text.strip()
    if not defaults_file_path.exists():
        raise FileNotFoundError(f"Could not find {defaults_file_path}")

    processed_defaults = _process_defaults(defaults_file_path)
    if strip:
        processed_defaults = _strip_defaults(processed_defaults)

    if not processed_defaults:
        raise ValueError(f"FC defaults file for {config} is empty after processing")

    defaults_out.parent.mkdir(parents=True, exist_ok=True)
    defaults_out.write_text('\n'.join(processed_defaults) + '\n')


def _write_fc_extra_hwdef(config: str, custom_version: str, out_path: Path) -> None:
    # Parse the XML file to get the extra_hwdef files
    xml_file = _get_config_index()[config]["xml_file"]
    root = ET.parse(xml_file).getroot()
    fc = root.find('flight_controller')
    if fc is None:
        raise ValueError(f"'flight_controller' element not found in {xml_file}")
    waf_extra_hwdef = fc.find('waf_extra_hwdef')
    if waf_extra_hwdef is None or waf_extra_hwdef.text is None:
        raise ValueError(f"'waf_extra_hwdef' element not found in {xml_file}")

    files = []
    for file in waf_extra_hwdef.text.split():
        path = CXPILOT_CONFIG_ROOT / file.strip()
        if not path.exists():
            raise FileNotFoundError(f"Extra hwdef file {file} in {config} does not exist")
        files.append(path)
    if not files:
        raise ValueError(f"No extra hwdef files found for flight controller in {config} in {xml_file}")

    # Write the extra_hwdef file
    _write_extra_hwdef(files, custom_version, out_path)


def _write_periph_extra_hwdef(files: list[Path], custom_version: str, out_path: Path) -> None:
    """
    Write the extra_hwdef file for periph firmware.

    This calls `write_extra_hwdef` to handle the include files and the
    AP_CUSTOM_FIRMWARE_STRING definition and then appends a placeholder
    CAN_APP_NODE_NAME definition to the end of the file. This is used to
    later to replace the board name in the final CPN firmware binary.
    """
    _write_extra_hwdef(files, custom_version, out_path)
    with out_path.open('a') as out_file:
        out_file.write("# Generated by " + str(Path(__file__).relative_to(CXPILOT_ROOT)) + "\n")
        out_file.write("undef CAN_APP_NODE_NAME\n")
        out_file.write(f"define CAN_APP_NODE_NAME \"{MAGIC_BOARD_NAME}\"\n")


def _write_extra_hwdef(files: list[Path], custom_version: str, out_path: Path) -> None:
    """
    Generate a combined extra_hwdef file for build.

    This file defines the AP_CUSTOM_FIRMWARE_STRING, to set the CxPilot version
    for printing on the GCS, and includes the contents of all the extra hwdef
    files specified in the files list. The files are included in the order they
    are provided, and each file's content is preceded by a comment indicating
    where it was imported from.

    Args:
        files (list[str]): List of paths to extra hwdef files to include.
        custom_version (str): value for AP_CUSTOM_FIRMWARE_STRING
        out_path (str): Path for the output file.
    """
    out_path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    lines.append(f"# Generated by {Path(__file__).relative_to(CXPILOT_ROOT)}")
    lines.append("# DO NOT EDIT THIS FILE DIRECTLY")
    lines.append(f"define AP_CUSTOM_FIRMWARE_STRING \"{custom_version}\"")
    for file in files:
        if not file.exists():
            raise FileNotFoundError(f"Extra hwdef file {file} does not exist")
        content = file.read_text()
        lines.append(f"# >>>> Imported from {file.relative_to(CXPILOT_ROOT)}")
        lines.append(content)
        lines.append(f"# <<<< End of {file.relative_to(CXPILOT_ROOT)}\n")
    out_path.write_text('\n'.join(lines) + '\n')


def _get_managed_cpns(config: str) -> list[ET.Element]:
    """
    Get the list of all managed CPN elements for a given configuration.

    This returns only the CPNs that must be built by us in CI. This is
    currently defined as those that have a 'waf_board' element

    Args:
        config (str): Name of the aircraft configuration.

    Returns:
        list[ET.Element]: List of CPN elements.
    """
    xml_file = _get_config_index()[config]["xml_file"]
    root = ET.parse(xml_file).getroot()
    cpn_list = root.find('cpn_list')
    if cpn_list is None:
        raise ValueError(f"'cpn_list' element not found in {xml_file}")
    out_list = []
    for cpn in cpn_list.findall('cpn'):
        waf_board = cpn.find('waf_board')
        if waf_board is None or waf_board.text is None:
            continue
        out_list.append(cpn)
    return out_list


def _get_cpn_extra_hwdefs(cpn: ET.Element) -> list[Path]:
    """
    Get the list of extra hwdef files for a given CPN element.

    Args:
        cpn (ET.Element): The CPN XML element.

    Returns:
        list[str]: List of paths to extra hwdef files.
    """
    waf_extra_hwdef = cpn.find('waf_extra_hwdef')
    if waf_extra_hwdef is None or waf_extra_hwdef.text is None:
        return []
    return [CXPILOT_CONFIG_ROOT / f.strip() for f in waf_extra_hwdef.text.split()]


def _get_cpn_base_firmware_name(cpn: ET.Element) -> str:
    """
    Get the base firmware name for a given CPN element.

    The base name is generated by combining the 'waf_board' name with the names
    of all the extra hwdef files, if any. The extra hwdef files are sorted
    alphabetically to ensure consistent naming, but if different CPNs have a
    different order of includes, we will throw an error later during the name
    conflict check.

    Args:
        cpn (ET.Element): The CPN XML element.

    Returns:
        str: The base firmware name.
    """
    waf_board = cpn.find('waf_board')
    if waf_board is None or waf_board.text is None:
        raise ValueError(f"'waf_board' element not found in CPN element: {ET.tostring(cpn, encoding='unicode')}")
    waf_extra_hwdef = _get_cpn_extra_hwdefs(cpn)
    if not waf_extra_hwdef:
        return waf_board.text.strip()
    names = [f.stem for f in waf_extra_hwdef]
    names.sort()  # Sort to ensure consistent naming
    return f"{waf_board.text.strip()}_{'_'.join(names)}"


def _process_defaults(file: Path, depth: int = 0) -> list[str]:
    """
    Process a defaults file, handling @include and @delete directives.
    """
    if depth > 10:
        raise Exception("Too many levels of @include")

    param_list = []
    lines = file.read_text().splitlines()
    for line in lines:
        if line.startswith("@include"):
            rel_path = line.split(maxsplit=1)[1]
            path = file.parent / rel_path.strip()
            param_list.extend(
                _process_defaults(path, depth + 1)
            )
            continue

        if line.startswith("@delete"):
            # Strip trailing comments
            line = re.sub(r"\s*#.*$", "", line)
            # Split into the required two parts
            line_split = line.split()
            if len(line_split) != 2:
                raise SyntaxError(f"Invalid @delete line in {file}: '{line}'")
            pattern = line_split[1]

            # Loop through the previously extracted parameters and comment out
            # the ones that match the pattern
            for i, param_line in enumerate(param_list):
                param_name = re.split(r"[\s,]+", param_line)[0]
                if fnmatch.fnmatch(param_name, pattern):
                    param_list[i] = "#deleted " + param_line
            # Now comment out this @delete directive
            line = "#" + line

        param_list.append(line)

    return param_list


def _strip_defaults(defaults: list[str]) -> list[str]:
    """
    Strip comments and whitespace from a list of default parameters and combine
    the separation of names and values to a single comma.

    Args:
        defaults (list[str]): List of default parameters.

    Returns:
        list[str]: Stripped list of default parameters.
    """
    stripped = []
    for line in defaults:
        # Remove comments
        line = re.sub(r"\s*#.*$", "", line)
        # Remove leading/trailing whitespace
        line = line.strip()
        if not line:
            continue
        # Split into name and value, then join them with a single comma
        parts = re.split(r"[\s,=]+", line)
        if len(parts) != 2:
            raise SyntaxError(f"Invalid parameter line: '{line}'")
        name, value = parts
        # Combine name and value with a single comma
        line = f"{name.strip()},{value.strip()}"
        stripped.append(line)
    return stripped


def _copy_scripts(config: str, dest_root: Path, symlink: bool = False) -> None:
    """
    Copy scripts for an aircraft configuration to the specified destination folder.

    Args:
        config (str): Name of the aircraft configuration to copy scripts for.
        dest_root (str): Destination directory to copy scripts to.
        symlink (bool): If True, use symlinks instead of copying files.
    """
    xml_file = _get_config_index()[config]["xml_file"]
    tree = ET.parse(xml_file)
    root = tree.getroot()
    lua_script_list = root.find('lua_script_list')
    if lua_script_list is None:
        raise ValueError(f"'lua_script_list' element not found in {xml_file}")

    # Construct a list of scripts to copy (or symlink)
    scripts = []  # type: list[tuple[Path, Path]]
    for lua_script in lua_script_list.findall('lua_script'):
        source_path = lua_script.find('source_path')
        if source_path is None or source_path.text is None:
            raise ValueError(f"'source_path' element not found in {xml_file}")
        source_path = CXPILOT_CONFIG_ROOT / source_path.text.strip()
        # "destination_path" is relative to the destination root
        dest_rel = lua_script.find('destination_path')
        if dest_rel is None or dest_rel.text is None:
            raise ValueError(f"'destination_path' element not found in {xml_file}")
        scripts.append((Path(source_path), Path(dest_rel.text.strip())))

    dest_root.mkdir(parents=True, exist_ok=True)

    for source_path, dest_rel in scripts:
        if not source_path.exists():
            raise FileNotFoundError(f"Lua script {source_path} does not exist")
        dest = dest_root / dest_rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        if symlink:
            if dest.is_symlink() or dest.exists():
                dest.unlink()
            dest.symlink_to(source_path)
        else:
            shutil.copy(source_path, dest)

        if not dest.exists():
            raise FileNotFoundError(f"Copied script {dest} does not exist")
        if not dest.lstat().st_size:
            raise ValueError(f"Copied script {dest} is empty")


@functools.lru_cache(maxsize=1)
def _get_periph_bases_all() -> dict:
    """
    Get all the unique periph firmware files that waf will need to build
    before the final CPN firmware files (with modified board names and defaults)
    can be generated.

    CPN build CI would take forever if we just rebuilt each CPN firmware each
    time, so we find the unique compiled output and hack the binaries with
    apj-tool to get the final CPN firmware files.

    We get these for all configurations, even deprecated ones, to enforce a key
    assumption that the name alone is enough to uniquely identify a build.
    """
    periph_bases = {}
    for config in _get_config_index().keys():
        for cpn in _get_managed_cpns(config):
            waf_board = cpn.find('waf_board')
            if waf_board is None or waf_board.text is None:
                raise ValueError(f"'waf_board' element not found in CPN element for {config}")
            waf_board = waf_board.text.strip()
            waf_extra_hwdef = _get_cpn_extra_hwdefs(cpn)
            base_name = _get_cpn_base_firmware_name(cpn)
            if base_name not in periph_bases:
                periph_bases[base_name] = {
                    "waf_board": waf_board,
                    "waf_extra_hwdef": waf_extra_hwdef,
                    "configs": set([config]),
                }
            else:
                # If we already have a base firmware by this name, confirm all
                # the values match. We'd have to be doing something really
                # strange in the XML files to get here, but if it happens, we
                # really need to know, as all sorts of nasty subtle errors will
                # happen if we don't catch this.
                existing = periph_bases[base_name]
                if existing["waf_board"] != waf_board:
                    raise ValueError(
                        f"{config} and {next(iter(existing['configs']))} generate different waf_board for {base_name}: "
                        f"{waf_board} vs {existing['waf_board']}"
                    )
                if existing["waf_extra_hwdef"] != waf_extra_hwdef:
                    raise ValueError(
                        f"{config} and {next(iter(existing['configs']))} generate different waf_extra_hwdef for {base_name}: "
                        f"{waf_extra_hwdef} vs {existing['waf_extra_hwdef']}"
                    )
                existing["configs"].add(config)

    return periph_bases


@functools.lru_cache(maxsize=1)
def _get_git_commit_ids() -> tuple[str, str, str]:
    """
    Return (integration_id, core_id, config_id) as 8-hex commit hashes with markers.

    Semantics:
      '*' means uncommitted changes exist. For the integration repo, this appears
      if the top-level worktree OR any submodule has uncommitted changes. This
      makes non-reproducible builds immediately obvious.

      '+' appears only on the integration ID and means submodules are at different
      commits than the recorded pointers, but everything is clean (no dirty state).

      In short:
      '*' says "reproducibility is blown - something is dirty"
      '+' says "clean but wrong commits - inspect XML for exact submodule SHAs"

    Hashes come from `git rev-parse --short=8 HEAD`. No enforcement is
    performed here; callers decide how to treat '*' and '+'. Repos are resolved
    relative to this script (CXPILOT_ROOT/CORE/CONFIG). Propagates
    subprocess.CalledProcessError on git failures.
    """
    def _get_hash(repo_path: Path) -> str:
        """Get the short commit hash for a given repository."""
        return subprocess.check_output(
            ['git', 'rev-parse', '--short=8', 'HEAD'],
            cwd=repo_path,
            text=True).strip()

    def _is_dirty(repo_path: Path, ignore_submodules: bool = False) -> bool:
        """Check if the repository is dirty."""
        args = ['git', 'status', '--porcelain']
        if ignore_submodules:
            args.append('--ignore-submodules=all')
        return bool(subprocess.check_output(args, cwd=repo_path, text=True).strip())

    def _submodules_sha_mismatch() -> bool:
        """Check if submodules are at different commits than the pinned pointers.

        Only checks commit SHA mismatch, not dirty state. Dirty state is handled
        separately to ensure any non-reproducible state gets a '*' marker.
        """
        # Get the pinned commit for cxpilot-core (full SHA)
        pinned_core = subprocess.check_output(
            ['git', 'ls-tree', 'HEAD', 'cxpilot-core'],
            cwd=CXPILOT_ROOT,
            text=True).split()[2]
        # Get checked out commit (full SHA for comparison)
        checked_out_core = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            cwd=CXPILOT_CORE_ROOT,
            text=True).strip()

        # Get the pinned commit for cxpilot-config (full SHA)
        pinned_config = subprocess.check_output(
            ['git', 'ls-tree', 'HEAD', 'cxpilot-config'],
            cwd=CXPILOT_ROOT,
            text=True).split()[2]
        # Get checked out commit (full SHA for comparison)
        checked_out_config = subprocess.check_output(
            ['git', 'rev-parse', 'HEAD'],
            cwd=CXPILOT_CONFIG_ROOT,
            text=True).strip()

        return (pinned_core != checked_out_core or pinned_config != checked_out_config)

    integration_commit_id = _get_hash(CXPILOT_ROOT)
    core_commit_id = _get_hash(CXPILOT_CORE_ROOT)
    config_commit_id = _get_hash(CXPILOT_CONFIG_ROOT)

    # Check if any repo is dirty - propagate '*' to integration for visibility
    # This makes non-reproducible builds immediately obvious
    if (_is_dirty(CXPILOT_ROOT, ignore_submodules=True) or
        _is_dirty(CXPILOT_CORE_ROOT) or
        _is_dirty(CXPILOT_CONFIG_ROOT)):
        integration_commit_id += '*'
    elif _submodules_sha_mismatch():
        integration_commit_id += '+'
    if _is_dirty(CXPILOT_CORE_ROOT):
        core_commit_id += '*'
    if _is_dirty(CXPILOT_CONFIG_ROOT):
        config_commit_id += '*'

    return integration_commit_id, core_commit_id, config_commit_id


def get_periph_env() -> dict:
    """
    Get environment variables for the embedding the git hash in the periph
    firmware during the build.
    """
    # We embed the integration commit ID as the ArduPilot hash, since this
    # alone (if clean; the case for all official builds) is enough to know the
    # state of all three repositories.
    #
    # Periph builds can't handle any additional markers in the environment
    # variables, so we strip them.
    integration_id, _, _ = _get_git_commit_ids()
    return _get_git_version_env_vars(integration_id.replace('*', '').replace('+', ''))


def _get_git_version_env_vars(git_version: str) -> dict:
    """
    Get environment variables to embed a commit hash into the firmware version
    """
    env = {}
    env['GIT_VERSION'] = git_version
    # Set the integer git version
    git_version_int = git_version.replace('+', '').replace('*', '')
    if len(git_version_int) < 8:
        raise ValueError("Need at least the first 32 bits of the git version to embed it as an integer")
    git_version_int = "0x" + git_version_int[:8]  # Take only the first 4 most significant bytes
    git_version_int = int(git_version_int, base=16)
    env['GIT_VERSION_INT'] = str(git_version_int)
    return env
