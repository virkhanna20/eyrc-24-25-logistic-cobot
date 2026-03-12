import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def generate_launch_description():
    # Get the AUV package share directory
    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_auv_sim').find('my_auv_sim')

    # Path to your AUV's SDF file
    sdf_file_auv = os.path.join(pkg_share, 'models', 'auv', 'auv_model.sdf')
    assert os.path.exists(sdf_file_auv), "The auv_model.sdf doesn't exist at " + str(sdf_file_auv)

    # Include the world launch file (you can create a similar one for starting Gazebo with your pool.sdf)
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('my_auv_sim'), 'launch', 'start_world_launch.py'),
        )
    )

    # Spawn AUV in Gazebo using the SDF file
    spawn_auv = launch_ros.actions.Node(
        package='gazebo_ros',
        name='auv_spawner',
        executable='spawn_entity.py',
        arguments=['-entity', 'auv', '-file', sdf_file_auv, '-x', '1.84', '-y', '-9.05', '-z', '0.1', '-Y', '3.14'],
        output='screen'
    )

 """   # Static Transform Publisher node (if needed for your robot)
    static_transform = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=["1.6", "-2.4", "-0.8", "3.14", "0", "0", "world", "odom"],
        output='screen'
    )"""

    # Launch description
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        start_world,
        spawn_auv,
        static_transform
    ])

