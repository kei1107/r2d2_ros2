memo

`colcon build --symlink-install --packages-select gazebo_r2d2`

`ros2 run xacro xacro -o [name] urdf/r2d2.urdf.xacro`

`rosdep install -i --from-paths src`

https://qiita.com/hakuturu583/items/7e3a278630422e17f0ba

https://gitlab.com/boldhearts/ros2_boldbot

http://wiki.ros.org/urdf/Tutorials/Using%20a%20URDF%20in%20Gazebo

ros2 control https://ros-controls.github.io/control.ros.org/index.html


https://github.com/ros-controls/ros2_controllers

https://github.com/ros-controls/ros2_control_demos


https://rt-net.jp/humanoid/archives/3571

https://pal-robotics.com/collaborative-projects/ros-control-ros2/

https://ros-controls.github.io/control.ros.org/index.html




```
ros2_control_node-6]   what():  According to the loaded plugin descriptions the class  with base class type hardware_interface::SystemInterface does not exist. Declared types are  fake_components/GenericSystem test_hardware_components/TestSystemCommandModes test_hardware_components/TestTwoJointSystem test_system

```

https://github.com/ros-controls/ros2_controllers

https://jeffzzq.medium.com/designing-a-ros2-robot-7c31a62c535a


https://github.com/ros-planning/navigation2/issues/1594

`ros2 run my_teleop_twist_keyboard my_teleop_twist_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel`


https://github.com/ros-controls/ros2_control_demos/blob/4888549d2f8ad6de9e025bdf62ad4541b6674729/ros2_control_demo_description/urdf/rrbot/ros2_control/rrbot_system_position_only.ros2_control.xacro


https://github.com/ros-simulation/gazebo_ros2_control
https://github.com/ros-simulation/gazebo_ros2_control/pull/44


---

```
[ros2-6] Traceback (most recent call last):
[ros2-6]   File "/opt/ros/foxy/bin/ros2", line 11, in <module>
[ros2-6]     load_entry_point('ros2cli==0.9.9', 'console_scripts', 'ros2')()
[ros2-6]   File "/opt/ros/foxy/lib/python3.8/site-packages/ros2cli/cli.py", line 67, in main
[ros2-6]     rc = extension.main(parser=parser, args=args)
[ros2-6]   File "/opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/command/control.py", line 37, in main
[ros2-6]     return extension.main(args=args)
[ros2-6]   File "/opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py", line 44, in main
[ros2-6]     if not args.state:
[ros2-6] AttributeError: 'Namespace' object has no attribute 'state'
```