from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os


# Launch file for starting all nodes
# TODO: Investigate node compositors and more efficient ways of bringing the system up

def generate_launch_description():

    web_control_server_node = Node(
        package='web_control_server',
        executable='server',
        name='server2',
        output='screen'
    )
    web_video_server_node = Node(
        package='web_video_server',
        namespace='web_video_server',
        executable='web_video_server'
        
    )

    motor_control_node = Node(
        package='motor_control_pkg',
        namespace='motor_control_pkg',
        executable='motor_control_node'
        
    )

    image_stitcher_node = Node(
        package='image_stitcher',
        namespace='stiched_images',
        executable='image_stitcher_node'
    )

    
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('twist_mux'),
                    'launch/twist_mux_launch.py'
                ])
            ])
        ),
        # motor_control_node,
        image_stitcher_node,
        # web_video_server_node,
        # web_control_server_node,
        
    ])
