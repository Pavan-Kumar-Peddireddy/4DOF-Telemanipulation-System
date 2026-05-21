import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Check for sim time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('bot'))
    # CHANGED: Look in 'urdf' folder instead of 'description'
    xacro_file = os.path.join(pkg_path, 'urdf', 'bot.urdf.xacro')
    
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        node_robot_state_publisher
    ])