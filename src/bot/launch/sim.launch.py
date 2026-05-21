import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'bot'
    pkg_share = get_package_share_directory(pkg_name)

    config_path = os.path.join(pkg_share, 'config', 'bot_controllers.yaml')

    # 1. NVIDIA GRAPHICS FIX (Cyborg 15)
    set_gpu_env = AppendEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_glx_env = AppendEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    
    # 2. MESH PATH FIX 
    # Points to the 'share' directory so Gazebo finds 'bot/meshes/...'
    resource_path = os.path.join(pkg_share, '..')
    set_gz_resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path)

    # 3. Include Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_share, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 4. Launch Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]), 
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'extra_args': f'--ros-args --params-file {config_path}'                   
        }.items()
    )

    # 5. Clock Bridge (Crucial for controllers)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/arm_controller/joint_trajectory@trajectory_msgs/msg/JointTrajectory]gz.msgs.JointTrajectory',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    # 6. Spawn the Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'bot', '-allow_renaming', 'true'],
        output='screen'
    )

    # 7. Controllers
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    return LaunchDescription([
        set_gpu_env,
        set_glx_env,
        set_gz_resource_path,
        rsp,
        gazebo,
        bridge,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller
    ])