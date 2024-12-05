from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

def generate_launch_description():
    
    input_uri = os.path.expanduser('~/ros_ws/src/rosbag2_tools/rosbag_analyze/rosbags/case_bag')
    output_uri = os.path.expanduser('~/ros_ws/src/rosbag2_tools/rosbag_analyze/created_rosbags/new')
    
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument('input_uri', 
                                        default_value=input_uri, 
                                        description='Input rosbag file URI'))
    ld.add_action(DeclareLaunchArgument('output_uri', 
                                        default_value=output_uri, 
                                        description='Output rosbag file URI'))
    ld.add_action(DeclareLaunchArgument('split_duration', 
                                        default_value='60', 
                                        description='Duration (in seconds) of each split'))

    
    rosbag_splitter = Node(
        package='rosbag_analyze',  # Name of the package
        executable='split_rosbag',  # Executable name (from entry_point)
        name='rosbag_splitter_node',
        output='screen',  # Output to the screen (console)
        parameters=[{
            'input_uri': LaunchConfiguration('input_uri'),
            'output_uri': LaunchConfiguration('output_uri'),
            'split_duration': LaunchConfiguration('split_duration')
        }]
    )
    
    ld.add_action(rosbag_splitter)
    
    return ld
