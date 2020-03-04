```
$ sudo apt-get install ros-melodic-joint-state-controller
$ sudo apt-get install ros-melodic-effort-controllers
$ sudo apt-get install ros-melodic-position-controllers
$ sudo apt-get install ros-melodic-pointcloud-to-laserscan
$ sudo apt-get install ros-melodic-teleop-twist-keyboard
```

```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=scout_robot/cmd_vel
```

xacro to URDF
```
$ rosrun xacro xacro scout.urdf.xacro > agilex_scout.urdf
```

URDF to PROTO
```
$ python urdf2webots.py --input=someRobot.urdf [--output=outputFile] [--box-collision] [--normal]
```

```
$ catkin_make_isolated --use-ninja --install
```

Cartographer

```
$ rosbag record -a
$ cartographer_rosbag_validate -bag_filename ./2019-09-24-03-59-01.bag
```

```
$ rosparam set use_sim_time true
$ rosbaly play <bag-name>
```

```
$ roslaunch scout_cartographer agilex_scout_offline_backpack_3d.launch bag_filenames:=${HOME}/Data/2019-10-07-11-49-39.bag
``` 

int32 CMD_ZOOM_IN = 0
int32 CMD_ZOOM_OUT = 1
int32 CMD_FOCUS_NEAR = 2
int32 CMD_FOCUS_FAR = 3
int32 CMD_SHUTTER_EXPAND = 4
int32 CMD_SHUTTER_SHRINK = 5

int32 CMD_MOVE_UP = 6
int32 CMD_MOVE_DOWN = 7
int32 CMD_MOVE_LEFT = 8
int32 CMD_MOVE_RIGHT = 9
int32 CMD_MOVE_UP_LEFT = 10
int32 CMD_MOVE_UP_RIGHT = 11
int32 CMD_MOVE_DOWN_LEFT = 12
int32 CMD_MOVE_DOWN_RIGHT = 13

param1 horizontal speed: 1-8
param2 vertical speed: 1-8

```
$ rosservice call /ipcam_ptz_control 6 0 2
```