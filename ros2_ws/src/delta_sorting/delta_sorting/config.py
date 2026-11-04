import json
from pathlib import Path
import yaml


def load_config(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("delta_sorting")) / "config/cell.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def load_geometry(path=None):
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("delta_sorting")) / "config/robot_geometry.json"
    return json.loads(Path(path).read_text())


def node_config(node):
    node.declare_parameter("config_file", "")
    return load_config(node.get_parameter("config_file").value or None)
