from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')
    weapon_dock_pkg = FindPackageShare('weapon_dock')
    joy_pkg = FindPackageShare('joy')

    # Gazebo Classic launch
    gz_launch_path = PathJoinSubstitution([gazebo_ros_pkg, 'launch', 'gazebo.launch.py'])
    
    # Classic uses GAZEBO_MODEL_PATH
    world_path = PathJoinSubstitution([weapon_dock_pkg, 'resource', 'worlds', 'robocon2026_world', 'world.sdf'])
    
    xacro_file = PathJoinSubstitution([weapon_dock_pkg, 'urdf', 'robot.urdf.xacro'])
    robot_description = Command(['xacro',' ', xacro_file])
    

    ld = LaunchDescription()

    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    
    
    ld.add_action(Node(
        package='weapon_dock',
        executable='js_convert_node',
        name='js_convert_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    ))
    

    ld.add_action(AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=PathJoinSubstitution([weapon_dock_pkg, 'resource', 'worlds'])
    ))

    ld.add_action(AppendEnvironmentVariable(
        name='IGN_FUEL_CONFIG_PATH',
        value=PathJoinSubstitution([weapon_dock_pkg, 'config', 'fuel_config.yaml'])
    ))

    # Include Gazebo Classic
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'world': world_path,
            'extra_gazebo_args': '--verbose'
        }.items(),
    ))

    # 发布 robot_description
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}  
        ]
    ))

    # Spawn robot in Gazebo Classic
    ld.add_action(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-z', '0.5', '-x', '4', '-y', '4'],
        output='screen'
    ))

    return ld