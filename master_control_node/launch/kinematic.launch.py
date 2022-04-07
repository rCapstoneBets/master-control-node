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
        Node(
            name="safety_pub",
            package="master_control_node",
            executable="safety_pub",
            output='screen',
        ),

        Node(
            name="kinematic_sol",
            package="master_control_node",
            executable="kinematic_sol",
            output='screen',
        ),
    ])