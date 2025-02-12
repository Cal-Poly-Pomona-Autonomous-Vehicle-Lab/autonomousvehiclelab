from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration
import os

# Launch file for starting all nodes
# TODO: Investigate node compositors and more efficient ways of bringing the system up

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    data_logger_node = Node(
        package='data_logger', 
        namespace='data_logger', 
        executable='data_logger_node', 
        parameters=[{'use_sim_time': use_sim_time}]
    )


    web_video_server_node = Node(
        package='web_video_server',
        namespace='web_video_server',
        executable='web_video_server',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    motor_control_node = Node(
        package='motor_control_pkg',
        namespace='motor_control_pkg',
        executable='motor_control_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    image_stitcher_node = Node(
        package='image_stitcher',
        namespace='stiched_images',
        executable='image_stitcher_node',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('twist_mux'),
                    'launch/twist_mux_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time
            }
        ),
        motor_control_node,
        image_stitcher_node,
        # web_video_server_node,
        # data_logger_node
    ])
