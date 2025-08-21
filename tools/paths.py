"""
Root paths for the cxpilot project.
"""
from pathlib import Path

CXPILOT_ROOT = Path(__file__).parents[2].resolve()
CXPILOT_CORE_ROOT = CXPILOT_ROOT / "cxpilot-core"
CXPILOT_CONFIG_ROOT = CXPILOT_ROOT / "cxpilot-config"
if not CXPILOT_CORE_ROOT.is_dir():
    raise FileNotFoundError(f"CXPILOT_CORE_ROOT does not exist: {CXPILOT_CORE_ROOT}")
if not CXPILOT_CONFIG_ROOT.is_dir():
    raise FileNotFoundError(f"CXPILOT_CONFIG_ROOT does not exist: {CXPILOT_CONFIG_ROOT}")
