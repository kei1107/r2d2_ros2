## r2d2_control4.launch.py (ros2_control_demo)

```shell
$ git clone https://github.com/ros-controls/ros2_control_demos
$ colcon_cd
$ colcon build --symlink-install
```


## r2d2_control6.launch.py (gazebo_ros2_control)

```shell
$ git clone https://github.com/ros-simulation/gazebo_ros2_control
$ cd gazebo_ros2_control
$ git checkout 956e7706338fd4fd1a2ef733ad54740d07e9a67a
$ colcon_cd
$ colcon build --symlink-install
$ source install/setup.bash


$ ros2 launch r2d2_control r2d2_control6.launch.py use_rviz:=false
$ colcon_cd r2d2_control

[WARN!!!!]
# https://github.com/ros-controls/ros2_control/commit/98650ab73110a751e81f423e0c4fa4931d905799
$ sudo cp /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py.old 
$ sudo cp ./extra/load_controller.py /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py

$ ros2 launch r2d2_control r2d2_control6.launch.py use_rviz:=false


$ ros2 run teleop_twiststamped_keyboard teleop_twiststamped_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel
```

---

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
