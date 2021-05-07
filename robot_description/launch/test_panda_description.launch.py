import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Get URDF via xacro
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'panda.xacro'
    )
    robot_description_config = xacro.process_file(
        robot_description_path,
        mappings={'end_effector': 'true', 'gazebo': 'false'}
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    joint_state_publisher_node = Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('robot_description'), 'rviz', 'panda.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
