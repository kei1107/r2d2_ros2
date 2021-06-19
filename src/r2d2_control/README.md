memo

`ros2 run teleop_twiststamped_keyboard teleop_twiststamped_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel`

---

https://qiita.com/hakuturu583/items/7e3a278630422e17f0ba

https://gitlab.com/boldhearts/ros2_boldbot

http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo

ros2 control https://ros-controls.github.io/control.ros.org/index.html


https://github.com/ros-controls/ros2_controllers

https://github.com/ros-controls/ros2_control_demos


https://rt-net.jp/humanoid/archives/3571

https://pal-robotics.com/collaborative-projects/ros-control-ros2/

https://ros-controls.github.io/control.ros.org/index.html





https://github.com/ros-controls/ros2_controllers

https://jeffzzq.medium.com/designing-a-ros2-robot-7c31a62c535a


https://github.com/ros-planning/navigation2/issues/1594

https://github.com/ros-controls/ros2_control_demos/blob/4888549d2f8ad6de9e025bdf62ad4541b6674729/ros2_control_demo_description/urdf/rrbot/ros2_control/rrbot_system_position_only.ros2_control.xacro


https://github.com/ros-simulation/gazebo_ros2_control
https://github.com/ros-simulation/gazebo_ros2_control/pull/44


https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp

---

```
ros2_control_node-6]   what():  According to the loaded plugin descriptions the class  with base class type hardware_interface::SystemInterface does not exist. Declared types are  fake_components/GenericSystem test_hardware_components/TestSystemCommandModes test_hardware_components/TestTwoJointSystem test_system
```

```
ubuntu@ubuntu-VirtualBox:~$ ros2 control list_controller_types 
controller_manager/test_controller                                     controller_interface::ControllerInterface
controller_manager/test_controller_failed_init                         controller_interface::ControllerInterface
controller_manager/test_controller_with_interfaces                     controller_interface::ControllerInterface
diff_drive_controller/DiffDriveController                              controller_interface::ControllerInterface
effort_controllers/JointGroupEffortController                          controller_interface::ControllerInterface
forward_command_controller/ForwardCommandController                    controller_interface::ControllerInterface
joint_state_broadcaster/JointStateBroadcaster                          controller_interface::ControllerInterface
joint_state_controller/JointStateController                            controller_interface::ControllerInterface
joint_trajectory_controller/JointTrajectoryController                  controller_interface::ControllerInterface
position_controllers/JointGroupPositionController                      controller_interface::ControllerInterface
velocity_controllers/JointGroupVelocityController                      controller_interface::ControllerInterface
```

---

> https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/fake_components_plugin_description.xml

Generic components for simple faking of system hardware.
