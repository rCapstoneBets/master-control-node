import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction

def generate_launch_description():

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

        # start up the master control node
        Node(
            name="master_control",
            package="master_control_node",
            executable="master_control",
            output='screen',
            # arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
        ),

        # start up the trajectory planner node
        Node(
            name="basic_trajectory",
            package="ball_trajectory_planner",
            executable="basic_trajectory_node",
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration("log_level")],
            parameters=[
                trajectory_config
            ] 
        ),

        # start up the master control node
        Node(
            name="act_test",
            package="master_control_node",
            executable="act_test",
            output='screen',
        ),

        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            output='screen',
            respawn=True,
            arguments=['--ros-args', '--log-level', 'ERROR'],
        ),  

        # spoof the hardware node 
        Node(
            name="hardware_node",
            package="tf2_ros",
            executable="static_transform_publisher",
            respawn=True,
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "lol", "whee"]
        ),
    ])