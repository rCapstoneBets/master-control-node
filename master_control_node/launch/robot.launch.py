import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction

def generate_launch_description():

    hardware_launch = PathJoinSubstitution([
        get_package_share_directory('hardware_controller'),
        'launch',
        'hardware.launch.py'
    ])

    ekf_config = PathJoinSubstitution([
        get_package_share_directory('master_control_node'),
        'cfg',
        'ekf_config.yaml'
    ])

    trajectory_config = PathJoinSubstitution([
        get_package_share_directory('ball_trajectory_planner'),
        'cfg',
        'basic_trajectory.yaml'
    ])

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            "log_level", 
            default_value="INFO",
            description="log level to use",
        ),

        # start up the hardware
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                hardware_launch
            ),
            launch_arguments=[
                ('log_level', LaunchConfiguration("log_level"))
            ]
        ),

        # start up the master control node
        Node(
            name="master_control",
            package="master_control_node",
            executable="master_control",
            respawn=True,
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
        ),

        # start up the trajectory planner node
        Node(
            name="basic_trajectory",
            package="ball_trajectory_planner",
            executable="basic_trajectory_node",
            respawn=True,
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            parameters=[
                trajectory_config
            ]
            
        ),

        # Publish world and odom as same thing until we get SLAM
        # This is here so we can compare ground truth from sim to odom
        Node(
            name="odom_to_world_broadcaster",
            package="tf2_ros",
            executable="static_transform_publisher",
            respawn=True,
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "world", "odom"]
        ),

        # start robot_localization Extended Kalman filter (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_localization_node',
            respawn=True,
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            parameters=[
                ekf_config,
                {                
                    'reset_on_time_jump': True,
                }
            ]
        ),

    ])