import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_prefix = get_package_share_directory('gazebo_r2d2')
    xacro_file = os.path.join(urdf_prefix,'urdf','r2d2.urdf.xacro')
    urdf_file = os.path.join(urdf_prefix,'urdf','r2d2.urdf')

    # load xacro
    doc = xacro.process_file(xacro_file)
    # generate urdf
    robot_desc = doc.toprettyxml(indent='  ')
    # write urdf file
    with open(urdf_file,'w') as f:
        f.write(robot_desc)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf_file]),
        Node(
            package='urdf_tutorial',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
    ])