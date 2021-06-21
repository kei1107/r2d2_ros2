# R2D2_ros2

This is a ROS2 package to test ros2_control and gazebo_ros2_control.  

I used a simple R2D2 model, customized with R2D2 model of  http://wiki.ros.org/urdf/Tutorials

---

## [WARNING!!]

**`ros2_control` and `gazebo_ros2_control` is still under development** at the time of writing this README.

If this repository doesn't work for you, please refer to the "latest" information.

- [ros2 control documents](https://ros-controls.github.io/control.ros.org/index.html)
- [ros2 control demos(github)](https://github.com/ros-controls/ros2_control_demos)
- [ros control roadmap](https://github.com/ros-controls/roadmap)
- [gazebo_ros2_control(github)](https://github.com/ros-simulation/gazebo_ros2_control)
---


There are 6 launch files in `r2d2_control` package.

|  launch  |  description  |
| ---- | ---- |
|  r2d2_control1.launch.py  |  Using libgazebo_ros_diff_drive, spawn from urdf file  |
|  r2d2_control2.launch.py  |  Using libgazebo_ros_diff_drive, spawn from /robot_description topic  |
|  r2d2_control3.launch.py  |  Using ros2_control  |
|  r2d2_control4.launch.py  |  Using ros2_control and DiffBotSystemHardware plugin. See [here](https://github.com/ros-controls/ros2_control_demos#example-4-differential-drive-mobile-robot) for DiffBotSystemHardware.   |
|  r2d2_control5.launch.py  |  Using ros2_control and my_r2d2_hardware plugin.  |
|  r2d2_control6.launch.py  |  Using ros2_control and gazebo_ros2_control  |


## Requirements  

- ROS2 foxy

### My Environment

```shell
$ uname -a
Linux ubuntu-VirtualBox 5.8.0-55-generic #62~20.04.1-Ubuntu SMP Wed Jun 2 08:55:04 UTC 2021 x86_64 x86_64 x86_64 GNU/Linux

$ cat /etc/lsb-release 
DISTRIB_ID=Ubuntu
DISTRIB_RELEASE=20.04
DISTRIB_CODENAME=focal
DISTRIB_DESCRIPTION="Ubuntu 20.04.2 LTS"

---

$ sudo apt show ros-foxy-ros2-control
Package: ros-foxy-ros2-control
Version: 0.6.1-1focal.20210601.154047
...

---

$ sudo apt show ros-foxy-ros2-controllers 
Package: ros-foxy-ros2-controllers
Version: 0.3.1-1focal.20210601.154705
...

---

$ sudo apt show ros-foxy-gazebo-ros
Package: ros-foxy-gazebo-ros
Version: 3.5.3-1focal.20210513.213411
...

---
```

## Build and Run

### Advance preparation

```shell
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/src
$ git clone https://github.com/kei1107/r2d2_ros2
$ rosdep install -i --from-paths r2d2_ros2
$ cd ..
$ colcon build --symlink-install 
```

### r2d2_control(1|2|3).launch.py

#### Build

Not necessary.

#### Run r2d2_control1.launch.py

```shell
[terminal 1]
$ source ~/dev_ws/install/setup.bash
$ ros2 launch r2d2_control r2d2_control1.launch.py use_rviz:=false

[terminal 2]
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Run r2d2_control2.launch.py

```shell
[terminal 1]
$ source ~/dev_ws/install/setup.bash
$ ros2 launch r2d2_control r2d2_control2.launch.py use_rviz:=false

[terminal 2]
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Run r2d2_control3.launch.py

```shell
[terminal 1]
$ source ~/dev_ws/install/setup.bash
$ ros2 launch r2d2_control r2d2_control3.launch.py use_rviz:=true

[terminal 2]
$ source ~/dev_ws/install/setup.bash
$ ros2 run teleop_twiststamped_keyboard teleop_twiststamped_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel
```

**Tips**

---

When controller_manager is stopped, the process may remain and affect the next execution.

---

If you want to use `teleop_twist_keyboard`, please make the following changes

```diff
--- a/r2d2_control/config/diff_drive_controller.yaml
+++ b/r2d2_control/config/diff_drive_controller.yaml
@@ -36,7 +36,7 @@ front_back_diff_drive_controller:
     # publish_limited_velocity: true
     # velocity_rolling_window_size: 10
 
-    # use_stamped_vel: false
+    use_stamped_vel: false
 
     # linear.x.has_velocity_limits: false
     # linear.x.has_acceleration_limits: false
```

After that, you can use `teleop_twist_keyboard`.

```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel_unstamped
```

### r2d2_control4.launch.py

#### Build (ros2_control_demos)

```shell
$ cd ~/dev_ws/src
$ git clone https://github.com/ros-controls/ros2_control_demos
$ cd ros2_control_demos
$ git checkout 2dc7c3717c1ceaaabd1576033d3f33b8c09882f6
$ rosdep install -i --from-paths . 
$ cd ../..
$ colcon build --symlink-install
```

#### Run r2d2_control4.launch.py

```shell
[terminal 1]
$ source ~/dev_ws/install/setup.bash
$ ros2 launch r2d2_control r2d2_control4.launch.py use_rviz:=true

[terminal 2]
$ source ~/dev_ws/install/setup.bash
$ ros2 run teleop_twiststamped_keyboard teleop_twiststamped_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel
```

**Tips**

---

When controller_manager is stopped, the process may remain and affect the next execution.

---

### r2d2_control5.launch.py

#### Build

Not necessary.

#### Run r2d2_control5.launch.py

```shell
[terminal 1]
$ source ~/dev_ws/install/setup.bash
$ ros2 launch r2d2_control r2d2_control5.launch.py use_rviz:=true

[terminal 2]
$ source ~/dev_ws/install/setup.bash
$ ros2 run teleop_twiststamped_keyboard teleop_twiststamped_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel
```

**Tips**

---

When controller_manager is stopped, the process may remain and affect the next execution.

---

### r2d2_control6.launch.py

#### Build

```shell
$ cd ~/dev_ws/src
$ git clone https://github.com/ros-simulation/gazebo_ros2_control
$ cd gazebo_ros2_control
$ git checkout 1b90ab1c4f28300e26d9f8c0a5bc56068e9abf55
$ rosdep install -i --from-paths . 
$ cd ../..
$ colcon build --symlink-install
```

#### Run r2d2_control6.launch.py

```shell
[terminal 1]
$ source ~/dev_ws/install/setup.bash
$ ros2 launch r2d2_control r2d2_control6.launch.py use_rviz:=false

[terminal 2]
$ source ~/dev_ws/install/setup.bash
$ ros2 run teleop_twiststamped_keyboard teleop_twiststamped_keyboard --ros-args -r cmd_vel:=/front_back_diff_drive_controller/cmd_vel
```

**Tips**

---

When gazebo or controller_manager is stopped, the process may remain and affect the next execution.

---

When run r2d2_control6.launch.py, the following warning occurs.

```
[ros2-6] deprecated warning: Please use 'load_controller --set_state start'
```

However, if you modify r2d2_control6.launch.py as follows, you get a different error.

```diff
--- a/r2d2_control/launch/r2d2_control6.launch.py
+++ b/r2d2_control/launch/r2d2_control6.launch.py
@@ -51,7 +51,7 @@ def generate_launch_description():
     #     output='screen'
     # )
     load_joint_state_controller = ExecuteProcess(
-        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_broadcaster'],
+        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
         output='screen'
     )
 
@@ -60,7 +60,7 @@ def generate_launch_description():
     #     output='screen'
     # )
     load_joint_diff_drive_controller = ExecuteProcess(
-        cmd=['ros2', 'control', 'load_start_controller', 'front_back_diff_drive_controller'],
+        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'front_back_diff_drive_controller'],        
         output='screen'
     )
```

```
[gzserver-2] [INFO] [1624244554.396891018] [controller_manager]: Loading controller 'joint_state_broadcaster'
[ros2-5] Traceback (most recent call last):
[ros2-5]   File "/opt/ros/foxy/bin/ros2", line 11, in <module>
[ros2-5]     load_entry_point('ros2cli==0.9.9', 'console_scripts', 'ros2')()
[ros2-5]   File "/opt/ros/foxy/lib/python3.8/site-packages/ros2cli/cli.py", line 67, in main
[ros2-5]     rc = extension.main(parser=parser, args=args)
[ros2-5]   File "/opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/command/control.py", line 37, in main
[ros2-5]     return extension.main(args=args)
[ros2-5]   File "/opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py", line 44, in main
[ros2-5]     if not args.state:
[ros2-5] AttributeError: 'Namespace' object has no attribute 'state'
[ERROR] [ros2-5]: process has died [pid 7250, exit code 1, cmd 'ros2 control load_controller --set-state start joint_state_broadcaster'].
[INFO] [ros2-6]: process started with pid [7282]
```

Next, you need the following fixes.

ref: https://github.com/ros-controls/ros2_control/commit/98650ab73110a751e81f423e0c4fa4931d905799

```shell
$ cd ~/dev_ws/src/r2d2_ros2/
$ sudo cp /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py.old 
$ sudo cp r2d2_control/extra/load_controller.py /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py
```

To be sure, please restore load_controller.py after checking the operation.

```shell
$ sudo cp /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py.old /opt/ros/foxy/lib/python3.8/site-packages/ros2controlcli/verb/load_controller.py
```
