import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # 1. SETUP VARIABLES AND PATHS
    pkg_name = 'serval'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    urdf_file_subpath = 'description/serval.urdf.xacro'
    xacro_file = os.path.join(pkg_share_dir, urdf_file_subpath)

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    controllers_yaml = os.path.join(pkg_share_dir, 'config', 'serval_control.yaml')
    
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        parameters=[{'dev': '/dev/input/js0'}],
        output='screen'
    )
    
    # Joint teleoperation node
    joint_teleop_node = Node(
        package='serval',
        executable='joint_teleop',
        output='screen'
    )

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # Joint states publisher (publishes /joint_states for TF tree)
    joint_states_node = Node(
        package='serval',
        executable='joint_states_publisher',
        output='screen'
    )

    # Return launch description
    return LaunchDescription([
        node_robot_state_publisher,
        joint_states_node,
        joy_node,
        joint_teleop_node,
    ])