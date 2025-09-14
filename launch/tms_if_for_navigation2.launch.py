from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    # Get the package directory
    zx200_description_dir = get_package_share_directory('zx200_description')

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    navigation2_navigate_through_poses_node = Node(
        package='tms_if_for_opera',
        executable='navigation2_navigate_through_poses',
        namespace='zx200',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        navigation2_navigate_through_poses_node,

    ])

    return ld