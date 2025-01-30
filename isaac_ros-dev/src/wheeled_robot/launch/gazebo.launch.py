import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'),
                'launch','gazebo.launch.py'])]),
        # launch_arguments={'world': PathJoinSubstitution([FindPackageShare('wheeled_robot'), 'world', 'yosemite.world'])}.items()
    )

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('wheeled_robot'), 'urdf', 'cybertruck.urdf.xacro'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # For RVIZ 
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    robot_spawn_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'robot'],
        output='screen'
    )
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_base_controller'],
        output='screen'
    )

    run_plotjuggler = ExecuteProcess(
            cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
            output='screen',
            shell=True  
    )
   
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        robot_spawn_node,
        joint_state_publisher_node, 
        load_joint_state_broadcaster,
        load_diff_drive_base_controller,
        run_plotjuggler
    ])