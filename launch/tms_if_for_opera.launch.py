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
    declare_robot_description_arg = DeclareLaunchArgument(
        'robot_description',
        default_value=str(zx200_description_dir + '/urdf/' + 'zx200.xacro'),
        description='Full path to the robot description file to load')

    declare_planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='manipulator',
        description='The planning group')

    declare_collision_object_component_name_arg = DeclareLaunchArgument(
        'collision_object_component_name',
        default_value='collision_objects_2',
        description='The collision object component name')

    # Use xacro package to convert xacro file to URDF
    robot_description_content = Command(['xacro ', LaunchConfiguration('robot_description')])

    # Create the backhoe_change_pose_action_server node
    backhoe_change_pose_action_server_node = Node(
        package='tms_if_for_opera',
        executable='backhoe_change_pose_action_server',
        parameters=[
            {'robot_description': robot_description_content},
            {'planning_group': LaunchConfiguration('planning_group')},
            {'collision_object_component_name': LaunchConfiguration('collision_object_component_name')}
        ])

    # Build the launch description
    ld = LaunchDescription([
        declare_robot_description_arg,
        declare_planning_group_arg,
        declare_collision_object_component_name_arg,
        backhoe_change_pose_action_server_node
    ])

    return ld