from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default = str(Path(get_package_share_directory("delta_sorting")) / "config/cell.yaml")
    config = LaunchConfiguration("config_file")
    arguments = [DeclareLaunchArgument("config_file", default_value=default),
                 DeclareLaunchArgument("max_attempts", default_value="24"),
                 DeclareLaunchArgument("report_dir", default_value="artifacts/latest")]
    nodes = []
    for executable in ["perception", "projection", "motion", "coordinator", "evaluate"]:
        params = {"use_sim_time": True, "config_file": config}
        if executable == "coordinator":
            params["max_attempts"] = ParameterValue(LaunchConfiguration("max_attempts"), value_type=int)
        if executable == "evaluate":
            params["report_dir"] = LaunchConfiguration("report_dir")
        nodes.append(Node(package="delta_sorting", executable=executable, parameters=[params], output="screen"))
    return LaunchDescription(arguments+nodes)
