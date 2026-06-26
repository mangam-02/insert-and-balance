"""Launch the peg-in-hole pipeline node with a couple of commonly-tuned parameters exposed.

This launches ONLY the pipeline. MoveIt, the camera, and FoundationPose must already be
running (see pipeline_node.py header). Example:
  ros2 launch peg_in_hole_pipeline peg_in_hole_pipeline.launch.py grasp_force:=30.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('grasp_force', default_value='20.0'),
        DeclareLaunchArgument('insertion_depth', default_value='0.08'),
        DeclareLaunchArgument('hole_approach_height', default_value='0.10'),
        DeclareLaunchArgument('vel_scale', default_value='0.1'),
    ]
    pipeline = Node(
        package='peg_in_hole_pipeline',
        executable='pipeline',
        name='peg_in_hole_pipeline',
        output='screen',
        parameters=[{
            'grasp_force': LaunchConfiguration('grasp_force'),
            'insertion_depth': LaunchConfiguration('insertion_depth'),
            'hole_approach_height': LaunchConfiguration('hole_approach_height'),
            'vel_scale': LaunchConfiguration('vel_scale'),
        }],
    )
    return LaunchDescription(args + [pipeline])
