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
        default_value='mst110cr')


    crawlerdump_navigate_through_poses_node = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_navigate_through_poses',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_navigate_anywhere_node = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_navigate_anywhere',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_follow_waypoints_node = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_follow_waypoints',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_swing_align_to_heading_node = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_swing_align_to_heading',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_release_soil_node = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_release_soil',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_swing_node = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_swing',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_name_arg,

        crawlerdump_navigate_through_poses_node,
        crawlerdump_navigate_anywhere_node,
        crawlerdump_follow_waypoints_node,
        crawlerdump_swing_align_to_heading_node,
        crawlerdump_release_soil_node,
        crawlerdump_swing_node,
    ])

    return ld