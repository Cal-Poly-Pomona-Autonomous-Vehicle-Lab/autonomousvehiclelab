from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("agv_car_description")

    nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "bicbot.rviz"])

    dual_ekf_config = PathJoinSubstitution([
        FindPackageShare("agv_car_description"), "config", "dual_ekf_gps_navsat.yaml"
    ])

    navsat_transform_config = PathJoinSubstitution([
        FindPackageShare("agv_car_description"), "config", "navsat_transform.yaml"
    ])


    ekf_node_filtered_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[dual_ekf_config],
        remappings=[('odometry/filtered', 'odometry/local')]
    )

    ekf_node_filtered_map = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[dual_ekf_config],
        remappings=[('odometry/filtered', 'odometry/global')]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_config],
        remappings=[
                    ('/imu', '/imu'),
                    ('gps/fix', 'gps/fix'), 
                    ('gps/filtered', 'gps/filtered'),
                    ('odometry/gps', 'odometry/gps'),
                    ('odometry/filtered', 'odometry/global')]           

    )

    nav2_nodes = [
        ekf_node_filtered_odom,
        ekf_node_filtered_map,
        navsat_transform_node,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'planner_server',
                    'controller_server',
                    'bt_navigator'
                ]
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ]

    return LaunchDescription(nav2_nodes)
