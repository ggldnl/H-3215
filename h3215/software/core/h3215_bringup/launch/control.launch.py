"""The control loop: the robot's description, ros2_control on the servo bus, the controllers.

    ros2 launch h3215_bringup control.launch.py                          the real servos
    ros2 launch h3215_bringup control.launch.py use_mock_hardware:=true  no hardware: every
                                                                         joint follows its command
robot.launch.py starts it with the rest of the robot. To set every leg joint at once:
    ros2 topic pub --once /leg_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [...]}"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description = FindPackageShare("h3215_description")
    bringup = FindPackageShare("h3215_bringup")

    args = [
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="simulate the servos instead of using the bus"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",
                              description="serial port of the servo adapter"),
        DeclareLaunchArgument("calibration_file",
                              default_value=PathJoinSubstitution([description, "config", "calibration.yaml"]),
                              description="servo offsets and directions"),
    ]

    robot_description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([description, "urdf", "h3215.urdf.xacro"]),
            " use_mock_hardware:=", LaunchConfiguration("use_mock_hardware"),
            " port:=", LaunchConfiguration("port"),
            " calibration_file:=", LaunchConfiguration("calibration_file"),
        ]),
        value_type=str,
    )

    return LaunchDescription(args + [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # Gets the description from robot_state_publisher's topic
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                PathJoinSubstitution([bringup, "config", "controllers.yaml"]),
                PathJoinSubstitution([description, "config", "joints.yaml"]),
            ],
            output="both",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "leg_position_controller"],
        ),
    ])
