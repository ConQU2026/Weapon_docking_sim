from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable # 引入追加动作
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ros_gz_sim_pkg = FindPackageShare('ros_gz_sim')
    weapon_dock_pkg = FindPackageShare('weapon_dock')

    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
    world_path = PathJoinSubstitution([weapon_dock_pkg, 'resource', 'worlds', 'robocon2026_world', 'world.sdf'])
    models_path = PathJoinSubstitution([weapon_dock_pkg, 'resource', 'models'])

    return LaunchDescription([
        
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=PathJoinSubstitution([weapon_dock_pkg])
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [world_path],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU'],
            remappings=[('/example_imu_topic', '/remapped_imu_topic')],
            output='screen'
        ),
    ])