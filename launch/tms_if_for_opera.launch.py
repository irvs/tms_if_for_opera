from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import Command, PathJoinSubstitution
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Get the package directory
    zx200_description_dir = get_package_share_directory('zx200_description')
    zx200_moveit_config_dir = get_package_share_directory('zx200_moveit_config')

    # Load kinematics.yaml
    kinematics_yaml = load_yaml('zx200_moveit_config', 'config/kinematics.yaml')
    
    # Declare the launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=str(zx200_description_dir + '/urdf/' + 'zx200.xacro'),
        description='Full path to the robot description file to load')

    declare_planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='manipulator',
        description='The planning group')

    declare_collision_object_record_name_arg = DeclareLaunchArgument(
        'collision_object_record_name',
        default_value='',
        description='The collision object record name')

    declare_collision_object_dump_record_name_arg = DeclareLaunchArgument(
        'collision_object_dump_record_name',
        default_value='',
        description='The collision object dump record names as comma-separated string (e.g., "record1,record2")')

    declare_other_robots_config_arg = DeclareLaunchArgument(
        'other_robots_config',
        default_value='',
        description='Other robots configuration for collision detection as comma-separated string (e.g., "mst110cr,robot2")')

    # Use xacro package to convert xacro file to URDF
    robot_description_content = Command(['xacro ', LaunchConfiguration('robot_description')])

    # Create the zx200_change_pose_action_server node
    excavator_change_pose_from_poses_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_change_pose_plan_from_poses',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            kinematics_yaml
    ])
    excavator_change_pose_plan_from_joint_values_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_change_pose_plan_from_joint_values',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            kinematics_yaml
    ])
    excavator_change_pose_execute_from_plan_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_change_pose_execute_from_plan',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            kinematics_yaml
    ])
    
    excavator_navigate_through_poses_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_navigate_through_poses',
        namespace='zx200',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    excavator_navigate_anywhere_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_navigate_anywhere',
        namespace='zx200',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])  

    excavator_follow_waypoints_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_follow_waypoints',
        namespace='zx200',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    excavator_assist_pose_to_joint_angles_node_zx200 = Node(
        package='tms_if_for_opera',
        executable='excavator_assist_excavation_pose_to_joint_angles',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            kinematics_yaml
        ])
    
    crawlerdump_navigate_through_poses_node_mst110cr_2 = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_navigate_through_poses',
        namespace='mst110cr_2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_navigate_anywhere_node_mst110cr_2 = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_navigate_anywhere',
        namespace='mst110cr_2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_follow_waypoints_node_mst110cr_2 = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_follow_waypoints',
        namespace='mst110cr_2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_swing_align_to_heading_node_mst110cr_2 = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_swing_align_to_heading',
        namespace='mst110cr_2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_release_soil_node_mst110cr_2 = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_release_soil',
        namespace='mst110cr_2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_swing_node_mst110cr_2 = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_swing',
        namespace='mst110cr_2',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_navigate_through_poses_node_mst2200vd = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_navigate_through_poses',
        namespace='mst2200vd',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])  
    
    crawlerdump_navigate_anywhere_node_mst2200vd = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_navigate_anywhere',
        namespace='mst2200vd',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_follow_waypoints_node_mst2200vd = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_follow_waypoints',
        namespace='mst2200vd',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    crawlerdump_swing_align_to_heading_node_mst2200vd = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_swing_align_to_heading',
        namespace='mst2200vd',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    crawlerdump_release_soil_node_mst2200vd = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_release_soil',
        namespace='mst2200vd',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]) 
    
    crawlerdump_swing_node_mst2200vd = Node(
        package='tms_if_for_opera',
        executable='crawlerdump_swing',
        namespace='mst2200vd',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])

    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_description_arg,
        declare_planning_group_arg,
        declare_collision_object_record_name_arg,
        declare_collision_object_dump_record_name_arg,
        declare_other_robots_config_arg,
        
        excavator_change_pose_from_poses_node_zx200,
        excavator_change_pose_plan_from_joint_values_node_zx200,
        excavator_change_pose_execute_from_plan_node_zx200,
        excavator_navigate_through_poses_node_zx200,
        excavator_navigate_anywhere_node_zx200,
        excavator_follow_waypoints_node_zx200,
        excavator_assist_pose_to_joint_angles_node_zx200,

        crawlerdump_navigate_through_poses_node_mst110cr_2,
        crawlerdump_navigate_anywhere_node_mst110cr_2,
        crawlerdump_follow_waypoints_node_mst110cr_2,
        crawlerdump_swing_align_to_heading_node_mst110cr_2,
        crawlerdump_release_soil_node_mst110cr_2,
        crawlerdump_swing_node_mst110cr_2,

        crawlerdump_navigate_through_poses_node_mst2200vd,
        crawlerdump_navigate_anywhere_node_mst2200vd,
        crawlerdump_follow_waypoints_node_mst2200vd,
        crawlerdump_swing_align_to_heading_node_mst2200vd,
        crawlerdump_release_soil_node_mst2200vd,
        crawlerdump_swing_node_mst2200vd,
        
    ])

    return ld