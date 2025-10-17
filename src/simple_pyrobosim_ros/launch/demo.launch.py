from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    ros_domain_id = os.environ.get("ROS_DOMAIN_ID", "0")

    # Declare launch argument for world file
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='/roscon25_ws/worlds/world1.yaml',
        description='Path to the world file for pyrobosim'
    )
    
    # Get the world file launch configuration
    world_file = LaunchConfiguration('world_file')
    
    # Include the pyrobosim_ros demo launch file with ROS_DOMAIN_ID=42
    pyrobosim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('pyrobosim_ros'),
                'launch',
                'demo.launch.py'
            ])
        ]),
        launch_arguments={
            'world_file': world_file
        }.items()
    )
    
    # Run the simple_pyrobosim_ros node in the default domain
    simple_pyrobosim_node = Node(
        package='simple_pyrobosim_ros',
        executable='simple_pyrobosim_ros',
        name='simple_pyrobosim_ros',
        output='screen'
    )
    
    return LaunchDescription([
        world_file_arg,
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        pyrobosim_launch,
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        simple_pyrobosim_node
    ])