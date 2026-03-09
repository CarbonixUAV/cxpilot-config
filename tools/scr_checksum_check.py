#!/usr/bin/env python3
"""Script checksum parameter validator, for CI and/or git hooks.

Verify that SCR_LD_CHECKSUM and SCR_RUN_CHECKSUM match the actual CRC of
each aircraft configuration's Lua scripts.

ArduPilot computes a per-script CRC32 (init=0, no final XOR, i.e.
CRC-32/JAMCRC), XORs all script CRCs together, then masks to 23 bits
(0x007FFFFF) before storing in the parameter.  Only top-level scripts are
included; modules loaded via require() are not checksummed.

The default (thorough) mode runs prepare_flight_controller() into a temp
directory for exact build parity.  Pass --quick to parse XML and defaults
directly without the full integration repo.

Usage:
    python cxpilot-config/tools/scr_checksum_check.py [--quick] [--config CONFIG ...]
"""

import sys
import re
import fnmatch
import binascii
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

CONFIG_ROOT = Path(__file__).resolve().parent.parent
AC_CONFIG_DIR = CONFIG_ROOT / "aircraft_configuration"

CHECKSUM_PARAM_MASK = 0x007FFFFF


# -- CRC --------------------------------------------------------------------

def _ardupilot_crc32(data: bytes) -> int:
    """Compute CRC-32/JAMCRC, matching ArduPilot's crc_crc32(0, buf, len)."""
    # Python's crc32 pre-conditions with 0xFFFFFFFF.  Passing 0xFFFFFFFF as the
    # initial value cancels out the pre-conditioning (0xFFFFFFFF ^ 0xFFFFFFFF = 0),
    # and the post-conditioning XOR is undone manually.
    return binascii.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


# -- Config index (standalone, no ac_config_tools dependency) ---------------

def _get_configs() -> dict[str, Path]:
    """Return {config_name: xml_path} for all active configs."""
    configs = {}
    for xml_file in sorted(AC_CONFIG_DIR.glob("*.xml")):
        root = ET.parse(xml_file).getroot()
        aircraft = root.find("aircraft")
        if aircraft is None:
            continue
        model = aircraft.findtext("model", "").strip()
        version = aircraft.findtext("model_version", "").strip()
        status = aircraft.findtext("status", "").strip()
        if not model or not version:
            continue
        if status != "active":
            continue
        configs[f"{model}_{version}"] = xml_file
    return configs


# -- Defaults processing (standalone reimplementation) ----------------------

def _process_defaults(file: Path, depth: int = 0) -> list[str]:
    """Process a defaults file, handling @include and @delete directives."""
    if depth > 10:
        raise Exception("Too many levels of @include")

    param_list = []
    for line in file.read_text().splitlines():
        if line.startswith("@include"):
            rel_path = line.split(maxsplit=1)[1].strip()
            param_list.extend(_process_defaults(file.parent / rel_path, depth + 1))
            continue

        if line.startswith("@delete"):
            line_clean = re.sub(r"\s*#.*$", "", line)
            parts = line_clean.split()
            if len(parts) != 2:
                raise SyntaxError(f"Invalid @delete line in {file}: '{line}'")
            pattern = parts[1]
            for i, param_line in enumerate(param_list):
                param_name = re.split(r"[\s,]+", param_line)[0]
                if fnmatch.fnmatch(param_name, pattern):
                    param_list[i] = "#deleted " + param_line
            line = "#" + line

        param_list.append(line)

    return param_list


def _extract_checksum_params(lines: list[str]) -> dict[str, int | None]:
    """Extract SCR_LD_CHECKSUM and SCR_RUN_CHECKSUM from processed param lines."""
    params: dict[str, int | None] = {
        "SCR_LD_CHECKSUM": None,
        "SCR_RUN_CHECKSUM": None,
    }
    for line in lines:
        line = re.sub(r"\s*#.*$", "", line).strip()
        if not line:
            continue
        parts = re.split(r"[\s,=]+", line)
        if len(parts) == 2 and parts[0] in params:
            params[parts[0]] = int(float(parts[1]))
    return params


# -- Quick mode -------------------------------------------------------------

def _scripts_for_config(xml_file: Path) -> list[tuple[Path, str]]:
    """Return [(source_path, destination_path), ...] for a config XML."""
    root = ET.parse(xml_file).getroot()
    script_list = root.find("lua_script_list")
    if script_list is None:
        return []
    scripts = []
    for entry in script_list.findall("lua_script"):
        src = entry.findtext("source_path", "").strip()
        dst = entry.findtext("destination_path", "").strip()
        if not src:
            raise ValueError(f"Empty source_path in {xml_file}")
        scripts.append((CONFIG_ROOT / src, dst))
    return scripts


def _compute_checksum_quick(scripts: list[tuple[Path, str]]) -> int:
    """Compute checksum from source files, skipping subdirectory destinations."""
    combined = 0
    for source_path, dest_rel in scripts:
        if '/' in dest_rel:
            continue
        combined ^= _ardupilot_crc32(source_path.read_bytes())
    return combined & CHECKSUM_PARAM_MASK


def _get_defaults_path(xml_file: Path) -> Path:
    """Return the defaults file path from a config XML."""
    root = ET.parse(xml_file).getroot()
    fc = root.find("flight_controller")
    if fc is None:
        raise ValueError(f"No flight_controller element in {xml_file}")
    defaults_el = fc.find("defaults_file")
    if defaults_el is None or defaults_el.text is None:
        raise ValueError(f"No defaults_file element in {xml_file}")
    return CONFIG_ROOT / defaults_el.text.strip()


def _check_params(
    config: str, computed: int, params: dict[str, int | None]
) -> list[str]:
    """Compare computed checksum against parameter values."""
    errors = []
    for param_name in ("SCR_LD_CHECKSUM", "SCR_RUN_CHECKSUM"):
        value = params[param_name]
        if value is None:
            errors.append(f"{config}: {param_name} not set in defaults (should be {computed})")
        elif value == -1:
            pass  # disabled
        elif value != computed:
            errors.append(
                f"{config}: {param_name} mismatch: param={value}, computed={computed}"
            )
    return errors


def check_config_quick(config: str, xml_file: Path) -> list[str]:
    """Check config by parsing XML and defaults directly."""
    scripts = _scripts_for_config(xml_file)
    if not scripts:
        return [f"{config}: no scripts defined"]

    computed = _compute_checksum_quick(scripts)
    defaults_path = _get_defaults_path(xml_file)
    lines = _process_defaults(defaults_path)
    params = _extract_checksum_params(lines)
    return _check_params(config, computed, params)


# -- Thorough mode (uses prepare_flight_controller) -------------------------

def check_config_thorough(config: str) -> list[str]:
    """Check config via prepare_flight_controller() for exact build parity.

    Requires the full integration repo structure (cxpilot-core must exist).
    """
    import tempfile

    sys.path.insert(0, str(Path(__file__).parent))
    import ac_config_tools  # noqa: E402

    with tempfile.TemporaryDirectory(prefix=f"checksum_{config}_") as tmp:
        tmp_path = Path(tmp)
        scripts_dir = tmp_path / "scripts"
        defaults_file = tmp_path / "defaults.parm"

        ac_config_tools.prepare_flight_controller(
            config=config,
            version="0.0.0-checksum-check",
            xml_out=tmp_path / "config.xml",
            scripts_out=scripts_dir,
            defaults_out=defaults_file,
            extra_hwdef_out=tmp_path / "extra_hwdef.dat",
            strip_defaults=True,
            symlink_scripts=True,
        )

        if not scripts_dir.exists():
            return [f"{config}: no scripts directory produced"]

        combined = 0
        for path in sorted(scripts_dir.iterdir()):
            if path.is_file() and path.suffix == '.lua':
                combined ^= _ardupilot_crc32(path.read_bytes())
        computed = combined & CHECKSUM_PARAM_MASK

        params: dict[str, int | None] = {
            "SCR_LD_CHECKSUM": None,
            "SCR_RUN_CHECKSUM": None,
        }
        for line in defaults_file.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = re.split(r"[\s,]+", line)
            if len(parts) == 2 and parts[0] in params:
                params[parts[0]] = int(float(parts[1]))

    return _check_params(config, computed, params)


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify script checksum parameters.")
    parser.add_argument(
        "--config", "-c",
        action="append",
        metavar="CONFIG",
        help="Check specific config(s) (default: all active configs).",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Quick mode: parse XML and defaults directly instead of using "
             "prepare_flight_controller(). Does not require the full "
             "integration repo.",
    )
    args = parser.parse_args()

    configs_index = _get_configs()

    if args.config:
        for name in args.config:
            if name not in configs_index:
                parser.error(f"Unknown config '{name}'. Available: {', '.join(sorted(configs_index))}")

    configs = args.config or sorted(configs_index)

    all_errors: list[str] = []
    for config in configs:
        if args.quick:
            errors = check_config_quick(config, configs_index[config])
        else:
            errors = check_config_thorough(config)
        if errors:
            for e in errors:
                print(f"FAIL  {e}")
            all_errors.extend(errors)
        else:
            print(f"OK    {config}")

    if all_errors:
        print(f"\n{len(all_errors)} checksum error(s) found.")
        return 1

    print("\nAll checksums OK.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
