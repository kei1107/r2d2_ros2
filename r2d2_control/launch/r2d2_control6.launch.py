import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, conditions
from launch.actions import ExecuteProcess,RegisterEventHandler, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration,Command
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_r2d2_control = get_package_share_directory('r2d2_control')
    xacro_file = os.path.join(pkg_r2d2_control,'urdf','r2d2_gazebo_ros2_control.urdf.xacro')
    robot_description = {'robot_description' : Command(['xacro', ' ', xacro_file])}

    rviz_file = os.path.join(pkg_r2d2_control,'config','r2d2_ros2_control.rviz')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, robot_description])

           
    spawn_entity = Node(
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
        ])

    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_broadcaster'],
        output='screen'
    )

    # load_joint_diff_drive_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'front_back_diff_drive_controller'],
    #     output='screen'
    # )
    load_joint_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'front_back_diff_drive_controller'],
        output='screen'
    )

    rviz = Node(    
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=conditions.IfCondition(use_rviz))

    return LaunchDescription([
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_diff_drive_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        rviz
    ])