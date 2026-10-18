import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# scipy and CasADi are in the image since devkit v2; FEB_PYDEPS is the old private install, kept as a fallback.
DEPS = os.environ.get("FEB_PYDEPS", "/home/autodrive_devkit/src/stack/.pydeps")


def generate_launch_description():
    params = os.path.join(get_package_share_directory("feb_cone_racer"), "config", "racer.yaml")
    # FEB_RACER_PARAMS names a second parameter file applied on top of the config, so a setting
    # can be tried on the car without editing the one everyone else runs.
    extra = os.environ.get("FEB_RACER_PARAMS", "")
    files = [params] + ([extra] if extra and os.path.exists(extra) else [])
    actions = [
        DeclareLaunchArgument("mode", default_value="trackdrive"),
        SetEnvironmentVariable("PYTHONPATH", DEPS + ":" + os.environ.get("PYTHONPATH", "")),
        Node(package="feb_cone_racer", executable="racer", name="racer",
             parameters=files + [{"use_sim_time": True}], output="screen", emulate_tty=True),
    ]
    try:                       # the team's cone ordering, when its package is in the stack too
        order = get_package_share_directory("feb_cone_ordering")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(order, "launch", "order.launch.py"))))
    except PackageNotFoundError:
        pass
    return LaunchDescription(actions)
