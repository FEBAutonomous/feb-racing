import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# scipy and CasADi are in the image since devkit v2; FEB_PYDEPS is the old private install, kept as a fallback.
DEPS = os.environ.get("FEB_PYDEPS", "/home/autodrive_devkit/src/stack/.pydeps")


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("feb_cone_racer"), "config")
    on_car = os.environ.get("FEB_CAR") == "1"
    files = [os.path.join(cfg, "racer.yaml")]
    # On the car (FEB_CAR=1) the clock is the wall clock and config/racer.car.yaml, if present, is
    # loaded on top: the real camera's numbers and the first-drive speeds.
    if on_car and os.path.exists(os.path.join(cfg, "racer.car.yaml")):
        files.append(os.path.join(cfg, "racer.car.yaml"))
    # FEB_RACER_PARAMS names one more parameter file applied on top of everything, so a setting
    # can be tried without editing the one everyone else runs.
    extra = os.environ.get("FEB_RACER_PARAMS", "")
    if extra and os.path.exists(extra):
        files.append(extra)
    actions = [
        DeclareLaunchArgument("mode", default_value="trackdrive"),
        SetEnvironmentVariable("PYTHONPATH", DEPS + ":" + os.environ.get("PYTHONPATH", "")),
        Node(package="feb_cone_racer", executable="racer", name="racer",
             parameters=files + [{"use_sim_time": not on_car}], output="screen", emulate_tty=True),
    ]
    try:                       # the team's cone ordering, when its package is in the stack too
        order = get_package_share_directory("feb_cone_ordering")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(order, "launch", "order.launch.py"))))
    except PackageNotFoundError:
        pass
    return LaunchDescription(actions)
