"""Launch the wrist-camera calibration node.

The MoveIt stack and the RealSense driver must already be running:

    ros2 launch franka_moveit_config moveit.launch.py robot_ip:=172.16.0.2 use_rviz:=true
    # + your RealSense driver

Then:

    ros2 launch franka_camera_calibration calibration.launch.py \
        poses_csv:=/abs/path/to/poses.csv

Override any other parameter from config/calibration.yaml on the command line,
e.g. image_topic:=/camera/color/image_raw
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('franka_camera_calibration')
    default_params = os.path.join(pkg, 'config', 'calibration.yaml')

    params_file = LaunchConfiguration('params_file')
    poses_csv = LaunchConfiguration('poses_csv')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Path to calibration parameter YAML.'),
        DeclareLaunchArgument('poses_csv', default_value='',
                              description='Absolute path to the Cartesian poses CSV.'),
        Node(
            package='franka_camera_calibration',
            executable='calibrate_wrist_camera',
            name='wrist_camera_calibrator',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'poses_csv': poses_csv}],
        ),
    ])
