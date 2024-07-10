import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #tracker_params_file = LaunchConfiguration(
    #'tracker_params_file',
    #default='param/map_info.yaml')
    tracker_params_file = get_package_share_directory('costmap_converter')+'/params/tracker.yaml'
    print(tracker_params_file)
    #declare_tracker_params_file = DeclareLaunchArgument(
    #'tracker_params_file',
    #default_value=tracker_params_file,
    #description='Full path to the ROS2 parameters file to use')
    dot_node = Node(
        package='costmap_converter',
        executable='standalone_converter',
        prefix=['xterm -e gdb -ex=r --args'],
        parameters=[tracker_params_file],
        output="screen")
    #ld = LaunchDescription()
    #ld.add_action(declare_tracker_params_file)
    return LaunchDescription(
        [
            dot_node,
        ]
    )