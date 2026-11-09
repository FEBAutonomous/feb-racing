import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# scipy and CasADi are in the image since devkit v2; FEB_PYDEPS is the old private install, kept as a fallback.
DEPS = os.environ.get("FEB_PYDEPS", "/home/autodrive_devkit/src/stack/.pydeps")


def racer_files(context):
    """The parameter files, later ones winning: racer.yaml, racer.car.yaml on the car, the venue
    profile (profile:=NAME -> config/venues/NAME.yaml, tuned on the dash's Tune page), FEB_RACER_PARAMS."""
    cfg = os.path.join(get_package_share_directory("feb_cone_racer"), "config")
    on_car = os.environ.get("FEB_CAR") == "1"
    files = [os.path.join(cfg, "racer.yaml")]
    if on_car and os.path.exists(os.path.join(cfg, "racer.car.yaml")):
        files.append(os.path.join(cfg, "racer.car.yaml"))
    profile = LaunchConfiguration("profile").perform(context)
    if profile:
        path = os.path.join(cfg, "venues", profile + ".yaml")
        if os.path.exists(path):
            files.append(path)
            print("racer: venue profile %s" % path)
        else:
            print("racer: no venue profile named %r (looked for %s); running without it" % (profile, path))
    extra = os.environ.get("FEB_RACER_PARAMS", "")
    if extra and os.path.exists(extra):
        files.append(extra)
    overrides = {"use_sim_time": not on_car}
    detector = LaunchConfiguration("detector").perform(context)
    if detector:
        overrides["detector"] = detector
    model = LaunchConfiguration("detector_model").perform(context)
    if model:
        overrides["detector_model"] = model
    return [Node(package="feb_cone_racer", executable="racer", name="racer",
                 parameters=files + [overrides], output="screen", emulate_tty=True)]


def generate_launch_description():
    actions = [
        DeclareLaunchArgument("mode", default_value="trackdrive"),
        DeclareLaunchArgument("profile", default_value="", description="venue profile in config/venues/"),
        DeclareLaunchArgument("detector", default_value="", description="hsv (default) or cnn"),
        DeclareLaunchArgument("detector_model", default_value="", description="ONNX file for detector:=cnn"),
        SetEnvironmentVariable("PYTHONPATH", DEPS + ":" + os.environ.get("PYTHONPATH", "")),
        OpaqueFunction(function=racer_files),
    ]
    try:                       # the team's cone ordering, when its package is in the stack too
        order = get_package_share_directory("feb_cone_ordering")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(order, "launch", "order.launch.py"))))
    except PackageNotFoundError:
        pass
    return LaunchDescription(actions)
