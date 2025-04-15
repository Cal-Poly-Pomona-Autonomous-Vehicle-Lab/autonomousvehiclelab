import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = FindPackageShare("agv_car_description")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg, "description", "final_agv_car.xacro"]),
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    rviz_config_file = PathJoinSubstitution([pkg, "rviz", "bicbot.rviz"])
    twist_mux_params = PathJoinSubstitution([pkg, "config", "twist_mux.yaml"])

    control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[
        {"update_rate": 1000},
        robot_description,
        robot_controllers,
    ],
    output="screen",
)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher_static',
    parameters=[{
        'source_list': ''  # disables listening to real joint_states
    }],
    output='screen')


    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bic_cont", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # twist_mux = Node(
    #     package='twist_mux',
    #     executable='twist_mux',
    #     parameters=[twist_mux_params],
    #     remappings=[('/cmd_vel', '/bic_cont/reference_unstamped')],
    #     output='screen',
    # )

    # lidar = IncludeLaunchDescription(...)
    # camera = IncludeLaunchDescription(...)
    # slam_toolbox = Node(...)
    # nav2 = IncludeLaunchDescription(...)

    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([pkg, "launch", "nav2_launch.py"])
    ]))


    return LaunchDescription([
        # joint_state_publisher_node,
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster,
        # twist_mux,
        # slam_toolbox,
        # nav2,
        # lidar,
        # camera,
        nav2_launch,
        
    ])
