"""The whole robot: the control loop, the depth camera and the high-level nodes.

    ros2 launch h3215_bringup robot.launch.py                          on the robot
    ros2 launch h3215_bringup robot.launch.py use_mock_hardware:=true  no hardware (no camera)

Then:
    ros2 service call /posture/stand std_srvs/srv/Trigger    torque on, move to the home pose
    ros2 service call /posture/relax std_srvs/srv/Trigger    torque off
    ros2 topic echo /battery
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = FindPackageShare("h3215_description")
    bringup = FindPackageShare("h3215_bringup")
    mock = LaunchConfiguration("use_mock_hardware")

    args = [
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="simulate the servos instead of using the bus"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",
                              description="serial port of the servo adapter"),
        DeclareLaunchArgument("calibration_file",
                              default_value=PathJoinSubstitution([description, "config", "calibration.yaml"]),
                              description="servo offsets and directions"),
        DeclareLaunchArgument("camera", default_value=PythonExpression(["'", mock, "' != 'true'"]),
                              description="start the depth camera (default: unless the hardware is mocked)"),
    ]

    core_parameters = [
        PathJoinSubstitution([description, "config", "joints.yaml"]),
        PathJoinSubstitution([bringup, "config", "core.yaml"]),
    ]

    return LaunchDescription(args + [
        IncludeLaunchDescription(
            PathJoinSubstitution([bringup, "launch", "control.launch.py"]),
            launch_arguments={
                "use_mock_hardware": mock,
                "port": LaunchConfiguration("port"),
                "calibration_file": LaunchConfiguration("calibration_file"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([bringup, "launch", "camera.launch.py"]),
            condition=IfCondition(LaunchConfiguration("camera")),
        ),
        Node(package="h3215_core", executable="posture", name="posture", parameters=core_parameters),
        Node(package="h3215_core", executable="monitor", name="monitor", parameters=core_parameters),
    ])
