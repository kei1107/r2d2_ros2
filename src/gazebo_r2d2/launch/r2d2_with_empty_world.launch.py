import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_gazebo_r2d2 = get_package_share_directory('gazebo_r2d2')
    xacro_file = os.path.join(pkg_gazebo_r2d2,'urdf','r2d2.urdf.xacro')
    urdf_file = os.path.join(pkg_gazebo_r2d2,'urdf','r2d2.urdf')

    # load xacro
    doc = xacro.process_file(xacro_file)
    # generate urdf
    robot_desc = doc.toprettyxml(indent='  ')
    # write urdf file
    with open(urdf_file,'w') as f:
        f.write(robot_desc)

    launch_file_dir = os.path.join(pkg_gazebo_r2d2,'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
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
                '-file', urdf_file,
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
    ])