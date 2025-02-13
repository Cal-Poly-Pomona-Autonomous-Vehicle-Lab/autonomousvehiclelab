#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('agv_car_description'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('agv_car_description'),
        'worlds',
        'empty.world'
    )

    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join('/home/agxorin1/autonomousvehiclelab/isaac_ros-dev/src/') 

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("agv_car_description"),
            "config",
            "controllers.yaml",
        ]
    )
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")
    
   
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'car', '-allow_renaming', 'true'],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",]
    )

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_broad'],
    #     output='screen'
    # )

    # # the steering controller libraries by default publish odometry on a separate topic than /tf
    # robot_ackermann_controller_spawner_remapped = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "ack_cont",
    #         "--param-file",
    #         robot_controllers,
    #         "--controller-ros-args",
    #         "-r /ack_cont/tf_odometry:=/tf",
    #     ],
    #     condition=IfCondition(remap_odometry_tf),
    # )

    robot_ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont", "--param-file", robot_controllers],
        # condition=UnlessCondition(remap_odometry_tf),
    )
    # load_ackermann_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'ack_cont'],
    #     output='screen'
    # )
    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    teleop_twist_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('/keyboard/cmd_vel', '/ack_cont/reference_unstamped')]  # Remap topic
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
        # DeclareLaunchArgument(
        #     "remap_odometry_tf",
        #     default_value="true",
        #     description="Remap odometry TF from the steering controller to the TF tree.",
        # ),
        bridge,
        # Launch gazebo  environment
        gzserver_cmd,
        gzclient_cmd,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[robot_ackermann_controller_spawner_remapped],
        #     )
        # ),
        robot_ackermann_controller_spawner,
        robot_state_publisher_cmd,
        gz_spawn_entity,
        teleop_twist_keyboard,
    ])

 