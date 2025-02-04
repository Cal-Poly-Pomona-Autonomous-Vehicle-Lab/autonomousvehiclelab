import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os
import launch_ros

def generate_launch_description():
     # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    pkg_share = launch_ros.substitutions.FindPackageShare(package='agv_bot_description').find('agv_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/agv_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
        # condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    start_gazebo_ros_spawner_cmd = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'agv_car',
                '-file', '/home/agxorin1/autonomousvehiclelab/isaac_ros-dev/src/agv_bot_description/src/description/agv_bot_description.urdf',
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01'
            ],
            output='screen',
    )

    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )

    # robot_localization_node = launch_ros.actions.Node(
    #    package='robot_localization',
    #    executable='ekf_node',
    #    name='ekf_filter_node',
    #    output='screen',
    #    parameters=[os.path.join('/workspaces/isaac_ros-dev/src/agv_bot_description', 'config/ekf.yaml')]
    # )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch/online_async_launch.py'
                ])
            ])              
        ),
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #                                     description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
                                            
        joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        
        robot_state_publisher_node,
        # start_gazebo_ros_spawner_cmd,
        # spawn_entity,
        # robot_localization_node,
        rviz_node
    ])