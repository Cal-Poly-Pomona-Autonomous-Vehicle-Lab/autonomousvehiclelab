from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os

def generate_launch_description():
    image_stitcher_node = Node(
         package='image_stitcher',
         namespace='stiched_images',
         executable='image_stitcher_node'
    )

    end_to_end = Node(
       package='end_to_end_CNN',
       namespace='end_to_end_CNN',
       executable='end_to_end_CNN_node',
       remappings=[
                ('/twist_mux/cmd_vel', '/twist_mux/cmd_vel'),
            ],
    )

     #teleop_twist_keyboard_node = Node(
    #    package='teleop_twist_keyboard',
    #    namespace='teleop_twist_keyboard',
    #    executable='teleop_twist_keyboard'
    # 	 )

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('realsense2_camera'),
        #             'launch/rs_launch.py'
        #         ])
        #     ])              
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('twist_mux'),
        #             'launch/twist_mux_launch.py'
        #         ])
        #     ])
        # ),
        image_stitcher_node, 
        end_to_end
        # object_avoidance_node.
        ])