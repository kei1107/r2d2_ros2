import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_r2d2_control = get_package_share_directory('r2d2_control')
    xacro_file = os.path.join(pkg_r2d2_control,'urdf','r2d2_ros2_control.urdf.xacro')
    robot_description = {'robot_description' : Command(['xacro', ' ', xacro_file])}

    robot_controller = 'front_back_diff_drive_controller'
    config_file = os.path.join(pkg_r2d2_control,'config','diff_drive_controller.yaml')

    rviz_file = os.path.join(pkg_r2d2_control,'config','r2d2_ros2_control.rviz')
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, robot_description]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        # controller
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, config_file],
            output={
                'stdout': 'screen',
                'stderr': 'screen'
            }
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            name="controller_spawner",
            arguments=[robot_controller]
        ),

        Node(    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=conditions.IfCondition(use_rviz)),
    ])