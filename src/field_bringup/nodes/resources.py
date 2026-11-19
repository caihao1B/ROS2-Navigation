from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import hashlib, json
import numpy as np
from math import isfinite

SHARE = Path(get_package_share_directory("field_bringup"))
CONFIG = SHARE / "config"
SCENARIO = SHARE / "scenarios"


def read(path):
    return json.loads(Path(path).read_text())


def verify_resources():
    manifest = read(CONFIG / "resources.json")
    for relative, expected in manifest.items():
        actual = hashlib.sha256((SHARE / relative).read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError("Inspection asset checksum mismatch: " + relative)
    return [SHARE / name for name in manifest]


def settings():
    return read(CONFIG / "inspection_rig.json")
