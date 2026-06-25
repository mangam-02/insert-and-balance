"""Launch the FoundationPose object-pose service.

Override any parameter on the command line, e.g.:
  ros2 launch franka_pose_estimation pose_service.launch.py prompt:=driller \
       mesh_file:=demo_data/driller/mesh/driller.obj
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = {
        'foundationpose_dir': '/workspace',
        'mesh_file': 'demo_data/nut/mesh/nut.obj',
        'prompt': 'nut',
        'camera_frame': 'camera_color_optical_frame',
        'zfar': '1.0',
        'est_refine_iter': '5',
        'track_refine_iter': '2',
    }
    declared = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    params = {k: LaunchConfiguration(k) for k in args}

    return LaunchDescription(declared + [
        Node(
            package='franka_pose_estimation',
            executable='foundationpose_pose_service.py',
            name='foundationpose_pose_service',
            output='screen',
            parameters=[params],
        ),
    ])
