import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions.find_executable import FindExecutable
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration,Command

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_r2d2_control = get_package_share_directory('r2d2_control')
    launch_file_dir = os.path.join(pkg_r2d2_control,'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'r2d2',
                '-x', '0',
                '-y', '0',
                '-z', '1',
                '-topic', '/robot_description'
            ]
        ),
    ])