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
from launch.actions import TimerAction


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

#     ros2_control_node = Node(
#     package="controller_manager",
#     executable="ros2_control_node",
#     parameters=[
#         {"update_rate": 1000},
#         robot_description,
#         robot_controllers,
#     ],
#     output="screen",
# )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    motor_control_node = Node(
    package="motor_control_pkg_car2",
    executable="motor_control_node",  
    name="motor_control_node",
    output="screen"
    )


    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    #     output="screen",
    # )

    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False}],
        output='screen'
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bic_cont", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # delay_robot_controller_spawner_after_joint_state_broadcaster = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[robot_controller_spawner],
    #     )
    # )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        # remappings=[('/cmd_vel_out', '/bic_cont/reference')],
        output='screen',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, "config", "slam_toolbox_params.yaml"])],
        remappings=[("/scan", "/velodyne_scan")],
    )
    
    kiss_icp_node = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        output='screen',
        remappings=[('pointcloud_topic', '/velodyne_points'),],
        parameters=[{
            'base_frame': 'base_link',
            'lidar_odom_frame': 'map',
            'publish_odom_tf': True,
            'invert_odom_tf': True,
            'max_range': 100.0,
            'min_range': 0.0,
            'deskew': True,
            'max_points_per_voxel': 20,
            'voxel_size': 1.0,
            'initial_threshold': 2.0,
            'min_motion_th': 0.1,
            'max_num_iteration': 500,
            'convergence_criterion': 0.0001,
            'max_num_threads': 0,
            'position_covariance': 0.1,
            'orientation_covariance': 0.1,
            'publish_debug_clouds': True,
            'use_sim_time': True
        }],
    )

    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([pkg, "launch", "nav2_launch.py"])
    ]))

    velodyne_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("velodyne"),
                    "launch",
                    "velodyne-all-nodes-VLP16-launch.py"
                ])
        ]),
        launch_arguments={
            "velodyne_ip": "192.168.1.201",
            "frame_id": "velodyne"}.items()
    )

    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.01,
            "min_height": -0.1,
            "max_height": 0.1,
            "angle_min": -3.14,
            "angle_max": 3.14,
            "angle_increment": 0.0087,  # ~0.5 deg
            "scan_time": 0.1,
            "range_min": 0.1,
            "range_max": 20.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "concurrency_level": 1
        }],
        remappings=[
            ("cloud_in", "/velodyne_points"),
            ("scan", "/scan")
        ]
    )

    pointcloud_to_scan_delayed = TimerAction(
        period=5.0,
        actions=[pointcloud_to_scan]
    )

    xsens_driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xsens_mti_ros2_driver"),
                "launch",
                "xsens_mti_node.launch.py"
            ])
        ]),
        launch_arguments={
            "device": "/dev/ttyUSB0"
        }.items()
    )


    ntrip_client_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare("ntrip"),
            "launch",
            "ntrip.launch"
        ])
    ])
)

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_pub_node,
        # ros2_control_node,
        # joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster,
        twist_mux,
        # nav2,
        # lidar,
        # camera,
        xsens_driver_node,
        # ntrip_client_node
        # slam_toolbox_node,
        # kiss_icp_node,
        motor_control_node,
        velodyne_launch,
        pointcloud_to_scan_delayed,
        nav2_launch,
        
    ])
