from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws/src/delta_sorting"))
from delta_sorting.config import load_config, load_geometry


@pytest.fixture
def cfg():
    return load_config(ROOT / "ros2_ws/src/delta_sorting/config/cell.yaml")


@pytest.fixture
def geometry():
    return load_geometry(ROOT / "assets/generated/robot_geometry.json")
