"""
radar_config.py - loads RF/geometry configuration presets for the TIDEP-01012
MIMO Cascade Radar from radar_configs/*.toml.

Why TOML: this repo already ships TOML config files for mimo.c (the
standalone C binary) under config/*.toml, parsed by toml/config.c. Using TOML
for the Python path too means presets are plain data files, editable without
touching any Python. (Not reusing config/*.toml directly - those follow a
simpler single-profile schema for the C binary; radar_configs/*.toml is the
full schema for mimo.py / mmwcas. See radar_configs/cascade_tx3_rx16.toml.)

Why this file exists (separate from mimo.py):
    mimo.py owns capture CONTROL FLOW. Radar CONFIGURATION (chirp profiles,
    idle times, TX/RX antenna geometry, frame timing) lives in
    radar_configs/*.toml.

Single source of truth: mmwcas.pyx C-struct defaults are ZEROED.
mmw_set_config() requires every field it applies from the selected TOML -
a missing key raises ValueError. mmw_init() refuses to run until
mmw_set_config() has succeeded.

Adding a new preset:
    1. Copy radar_configs/cascade_tx3_rx16.toml (or another preset), rename it
       (e.g. radar_configs/wide_idle.toml -> preset name "wide_idle").
    2. Edit only the fields that differ.
    3. Select it:  python3 mimo.py --radar-config <name>

mimo.channel.rxChannelEn may be a scalar (broadcast to all 4 devices) or a
length-4 list [Dev1, Dev2, Dev3, Dev4] for per-device RX enable
(see cascade_tx3_rx8.toml).
"""
import os

try:
    import tomllib                          # Python 3.11+
except ModuleNotFoundError:
    try:
        import tomli as tomllib             # Python < 3.11
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            "Need a TOML reader: Python 3.11+ has tomllib in the stdlib; "
            "on older Python install tomli with:  pip install tomli"
        ) from e

_HERE = os.path.dirname(os.path.abspath(__file__))

# Directory of *.toml presets (filename stem = --radar-config name).
# Not the same as config/*.toml (mimo.c / ./mmwave schema).
RADAR_CONFIG_DIR = os.path.join(_HERE, "radar_configs")

# Built-in default preset name (radar_configs/cascade_tx3_rx16.toml).
DEFAULT_RADAR_CONFIG = "cascade_tx3_rx8_3rps"


def _load_all_presets(directory=RADAR_CONFIG_DIR):
    """Load every *.toml file in `directory` into {name: config_dict},
    keyed by filename stem (e.g. cascade_tx3_rx16.toml -> "cascade_tx3_rx16")."""
    if not os.path.isdir(directory):
        raise RuntimeError(f"Radar config directory not found: {directory}")
    presets = {}
    for fname in sorted(os.listdir(directory)):
        if not fname.endswith(".toml"):
            continue
        name = fname[: -len(".toml")]
        path = os.path.join(directory, fname)
        with open(path, "rb") as f:
            presets[name] = tomllib.load(f)
    if not presets:
        raise RuntimeError(f"No .toml radar config presets found in {directory}")
    return presets


RADAR_CONFIGS = _load_all_presets()

# Convenience for callers that want the built-in default preset.
config_dict = RADAR_CONFIGS[DEFAULT_RADAR_CONFIG]


def get_radar_config(name=None):
    """Look up a named preset from RADAR_CONFIGS.
    Defaults to DEFAULT_RADAR_CONFIG ('cascade_tx3_rx16')."""
    if name is None:
        name = DEFAULT_RADAR_CONFIG
    try:
        return RADAR_CONFIGS[name]
    except KeyError:
        valid = ", ".join(sorted(RADAR_CONFIGS))
        raise ValueError(f"Unknown radar config '{name}'. Valid options: {valid}")
