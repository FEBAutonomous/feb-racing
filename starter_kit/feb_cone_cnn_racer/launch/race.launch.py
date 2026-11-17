"""feb_cone_racer with the trained detector: everything the same, the camera colours from the model.
Arguments: mode (from the card), profile (unused by the detector but accepted), detector_model
(ONNX path; empty by default so the racer config decides: cones.onnx on a PC, cones_n416.onnx on the Jetson)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = os.path.join(get_package_share_directory("feb_cone_racer"), "launch", "race.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="trackdrive"),
        DeclareLaunchArgument("profile", default_value=""),
        DeclareLaunchArgument("detector_model", default_value="", description="ONNX path; empty = the racer config (racer.car.yaml picks the Jetson model)"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(base),
                                 launch_arguments={"mode": LaunchConfiguration("mode"), "profile": LaunchConfiguration("profile"),
                                                   "detector": "cnn", "detector_model": LaunchConfiguration("detector_model")}.items()),
    ])
