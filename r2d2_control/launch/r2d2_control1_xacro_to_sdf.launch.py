import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import ExecuteProcess,RegisterEventHandler, IncludeLaunchDescription,LogInfo
from launch.launch_context import LaunchContext
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_r2d2_control = get_package_share_directory('r2d2_control')
    xacro_file = os.path.join(pkg_r2d2_control,'urdf','r2d2_ros_control.urdf.xacro')
    urdf_file = os.path.join(pkg_r2d2_control,'urdf','r2d2_ros_control.urdf')
    sdf_file = os.path.join(pkg_r2d2_control,'urdf','r2d2_ros_control.sdf')

    # generate urdf
    urdf_robot_desc = Command(['xacro', ' ', xacro_file]).perform(LaunchContext())
    # write urdf file
    with open(urdf_file,'w') as f:
        f.write(urdf_robot_desc)
    sdf_robot_desc = Command(['gz', ' ', 'sdf', ' ', '-p', ' ', urdf_file]).perform(LaunchContext());
    with open(sdf_file,'w') as f:
        f.write(sdf_robot_desc)

    launch_file_dir = os.path.join(pkg_r2d2_control,'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    rviz_file = os.path.join(pkg_r2d2_control,'config','r2d2_ros_control.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher_for_r2d2_control_1_and_2.launch.py']),
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
                '-file', sdf_robot_desc,
            ]
        ),
        Node(    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=conditions.IfCondition(use_rviz)),
        ])