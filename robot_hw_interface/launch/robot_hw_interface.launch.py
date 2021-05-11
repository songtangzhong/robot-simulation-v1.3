import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

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

    robot_controller = os.path.join(
        get_package_share_directory('robot_hw_interface'),
        'config',
        'robot_controllers.yaml'
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('robot_hw_interface'),
        'rviz',
        'panda.rviz'
    )

    control_node = Node(
      package='controller_manager',
      executable='ros2_control_node',
      parameters=[robot_description, robot_controller],
      output={
          'stdout': 'screen',
          'stderr': 'screen',
        },
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    start_controller_node = Node(
        package='robot_hw_interface',
        executable='start_controller',
        output='screen',
    )

    trigger_state_sub_node = Node(
        package='robot_fun',
        executable='trigger_state_sub',
        output='screen',
    )

    '''rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
    )'''

    return LaunchDescription([
        control_node,
        start_controller_node,
        robot_state_publisher_node,
        trigger_state_sub_node,
    ])
