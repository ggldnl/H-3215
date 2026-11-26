"""The depth camera, an Intel RealSense D435i: depth, color and its IMU.

    ros2 launch h3215_bringup camera.launch.py

Topics under /camera/camera/: depth/image_rect_raw, color/image_raw, imu (gyro and accelerometer
in one message). Its frames hang from camera_link, which the robot's description places on the
base.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 424 x 240 at 30 Hz: enough for an elevation map around the robot, light on the Pi's USB and CPU
PROFILE = "424,240,30"


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]),
            launch_arguments={
                "camera_name": "camera",
                "depth_module.depth_profile": PROFILE,
                "rgb_camera.color_profile": PROFILE,
                "enable_gyro": "true",
                "enable_accel": "true",
                "unite_imu_method": "2",  # one IMU message, accelerometer interpolated to the gyro
                "align_depth.enable": "false",
                "pointcloud.enable": "false",
            }.items(),
        ),
    ])
