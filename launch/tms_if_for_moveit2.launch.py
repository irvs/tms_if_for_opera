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
    moveit2_change_pose_action_server_node = Node(
        package='tms_if_for_opera',
        executable='moveit2_change_pose_action_server',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    moveit2_change_pose_plan_action_server_node = Node(
        package='tms_if_for_opera',
        executable='moveit2_change_pose_plan_action_server',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    moveit2_excavate_simple_action_server_node = Node(
        package='tms_if_for_opera',
        executable='moveit2_excavate_simple_action_server',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    moveit2_excavate_simple_plan_action_server_node = Node(
        package='tms_if_for_opera',
        executable='moveit2_excavate_simple_plan_action_server',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    moveit2_release_simple_action_server_node = Node(
        package='tms_if_for_opera',
        executable='moveit2_release_simple_action_server',
        namespace='zx200',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')},
            {'collision_object_dump_record_name': LaunchConfiguration('collision_object_dump_record_name')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    scene_manager_node = Node(
        package='tms_if_for_opera',
        executable='scene_manager',
        namespace='zx200',
        parameters=[
            {'planning_group': LaunchConfiguration('planning_group')},
            {'other_robots_config': LaunchConfiguration('other_robots_config')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ])
    
    # backhoe_excavate_simple_action_server_node = Node(
    #     package='tms_if_for_opera',
    #     executable='backhoe_excavate_simple_action_server',
    #     parameters=[
    #         {'robot_description': robot_description_content},
    #         {'planning_group': LaunchConfiguration('planning_group')},
    #         {'collision_object_record_name': LaunchConfiguration('collision_object_record_name')}
    #     ])

    # Build the launch description
    ld = LaunchDescription([
        declare_use_sim_time_arg,
        declare_robot_description_arg,
        declare_planning_group_arg,
        declare_collision_object_record_name_arg,
        declare_collision_object_dump_record_name_arg,
        declare_other_robots_config_arg,
        
        moveit2_change_pose_action_server_node,
        moveit2_change_pose_plan_action_server_node,
        moveit2_excavate_simple_action_server_node,
        moveit2_excavate_simple_plan_action_server_node,
        moveit2_release_simple_action_server_node,
        # backhoe_excavate_simple_action_server_node
        scene_manager_node,
    ])

    return ld