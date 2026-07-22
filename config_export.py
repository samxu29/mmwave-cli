"""
config_export.py - thin wrapper around utility.export_config_to_json.

Canonical exporter lives in utility.py and reads ALL RF/geometry fields from
config_dict (no parallel hardcodes). Prefer:

    from utility import export_config_to_json
"""
from utility import export_config_to_json

__all__ = ["export_config_to_json"]
