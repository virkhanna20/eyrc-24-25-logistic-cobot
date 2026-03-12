import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare the launch argument for the world file
        DeclareLaunchArgument(
            'world_file',
            default_value='$(find my_auv_sim)/worlds/pool.sdf',
            description='Full path to the world file to load'
        ),

        # Include Gazebo server node with the world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen',
            arguments=[LaunchConfiguration('world_file')]
        ),

        # Spawn the AUV model in the Gazebo simulation
        Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_auv',
            arguments=[
                '-file', '$(find my_auv_sim)/models/auv.sdf',
                '-sdf', '-model', 'auv_model'
            ],
            output='screen',
        ),

        # Log info when the simulation starts
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('world_file', '$(find my_auv_sim)/worlds/pool.sdf'),
            msg="Gazebo world with pool.sdf and AUV model spawned successfully!"
        ),
    ])

