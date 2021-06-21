Notes.

Reference Links
- https://rt-net.jp/humanoid/archives/3571
- https://jeffzzq.medium.com/designing-a-ros2-robot-7c31a62c535a
- https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/diff_drive_controller.cpp

---

Available hardware interfaces

```
ros2_control_node-6]   what():  According to the loaded plugin descriptions the class  with base class type hardware_interface::SystemInterface does not exist. Declared types are  fake_components/GenericSystem test_hardware_components/TestSystemCommandModes test_hardware_components/TestTwoJointSystem test_system
```

---

Available controllers

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

description of fake_components/GenericSystem

> https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/fake_components_plugin_description.xml

```xml
<description>
    Generic components for simple faking of system hardware.
</description>
```