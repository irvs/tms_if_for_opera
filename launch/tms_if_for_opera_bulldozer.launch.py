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
    declare_monitor_enabled_arg = DeclareLaunchArgument(
        'monitor_enabled', default_value='false',
        description='Enable robot-state communication monitoring')
    declare_monitor_type_arg = DeclareLaunchArgument(
        'monitor_type', default_value='pose',#'joint_state',
        description='Monitor type: pose or joint_state (pose supports PoseStamped or Odometry)')
    declare_monitor_msg_type_arg = DeclareLaunchArgument(
        'monitor_msg_type', default_value='nav_msgs/msg/Odometry',#'geometry_msgs/msg/PoseStamped',
        description='Pose message type: geometry_msgs/msg/PoseStamped or nav_msgs/msg/Odometry')
    declare_monitor_topic_arg = DeclareLaunchArgument(
        'monitor_topic', default_value='global_pose',#'joint_states',
        description='Robot-state topic to monitor (relative names follow node namespace)')
    declare_monitor_timeout_arg = DeclareLaunchArgument(
        'monitor_timeout', default_value='3.0',#'1.0',
        description='Communication timeout in seconds')
    declare_monitor_check_period_arg = DeclareLaunchArgument(
        'monitor_check_period', default_value='0.1',
        description='Communication monitor check period in seconds')
    monitor_parameters = {
        'monitor_enabled': LaunchConfiguration('monitor_enabled'),
        'monitor_type': LaunchConfiguration('monitor_type'),
        'monitor_msg_type': LaunchConfiguration('monitor_msg_type'),
        'monitor_topic': LaunchConfiguration('monitor_topic'),
        'monitor_timeout': LaunchConfiguration('monitor_timeout'),
        'monitor_check_period': LaunchConfiguration('monitor_check_period'),
    }

    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='d37pxi_24')

    bulldozer_blade_control_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_blade_control',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}, monitor_parameters])

    bulldozer_navigate_through_poses_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_navigate_through_poses',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}, monitor_parameters])

    bulldozer_navigate_anywhere_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_navigate_anywhere',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}, monitor_parameters])

    bulldozer_follow_waypoints_node = Node(
        package='tms_if_for_opera',
        executable='bulldozer_follow_waypoints',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}, monitor_parameters])


    # Build the launch description
    ld = LaunchDescription([
        declare_monitor_enabled_arg,
        declare_monitor_type_arg,
        declare_monitor_msg_type_arg,
        declare_monitor_topic_arg,
        declare_monitor_timeout_arg,
        declare_monitor_check_period_arg,

        declare_use_sim_time_arg,
        declare_robot_name_arg,

        bulldozer_blade_control_node,
        bulldozer_navigate_through_poses_node,
        bulldozer_navigate_anywhere_node,
        bulldozer_follow_waypoints_node
    ])

    return ld
