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

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='d37pxi_24')

    bulldozer_navigate_anywhere_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_blade_control',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    bulldozer_navigate_through_poses_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_navigate_through_poses',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    bulldozer_navigate_anywhere_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_navigate_anywhere',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    bulldozer_follow_waypoints_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_follow_waypoints',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])


    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,

        bulldozer_navigate_through_poses_node,
        bulldozer_navigate_anywhere_node,
        bulldozer_follow_waypoints_node
    ])

    return ld