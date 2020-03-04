# ROS Packages for Scout Mobile Base

## Packages

* scout_bringup: launch and configuration files to start ROS nodes 
* scout_base: a ROS wrapper around Scout SDK to monitor and control the robot
* scout_sdk: Scout SDK customized for ROS
* scout_msgs: scout related message definitions
* (scout_ros: meta package for the Scout robot ROS packages)

## Communication interface setup

### Setup CAN-To-USB adapter 
 
1. Enable gs_usb kernel module
   
    ```
    $ sudo modprobe gs_usb
    ```

2. Bringup can device
   
   ```
   $ sudo ip link set can0 up type can bitrate 500000
   ```

3. If no error occured during the previous steps, you should be able to see the can device now by using command
   
   ```
   $ ifconfig -a
   ```

4. Install and use can-utils to test the hardware
   
    ```
    $ sudo apt install can-utils
    ```

5. Testing command
   
    ```
    # receiving data from can0
    $ candump can0
    # send data to can0
    $ cansend can0 001#1122334455667788
    ```

Two scripts inside the "scout_bringup/scripts" folder are provided for easy setup. You can run "./setup_can2usb.bash" for the first-time setup and run "./bringup_can2usb.bash" to bring up the device each time you unplug and re-plug the adapter.

### Setup UART

Generally your UART2USB cable should be automatically recognized as "/dev/ttyUSB0" or something similar and ready for use. If you get the error "... permission denied ..." when trying to open the port, you need to grant access of the port to your user accout:

```
$ sudo usermod -a -G dialout <username>
```

Replace "<username>" in the above command with your Linux username. You need to re-login to get the change to take effect.

## Basic usage of the ROS package

1. Install dependent library and ROS packages

    ```
    $ sudo apt install libpcap-dev
    ```

    ```
    $ sudo apt install ros-melodic-teleop-twist-keyboard
    $ sudo apt install ros-melodic-navigation
    $ sudo apt install ros-melodic-pointcloud-to-laserscan
    $ sudo apt install ros-melodic-video-stream-opencv
    ```

    Change ros-melodic-* in the command to ros-kinetic-* if you're using ROS Kinetic.

2. Clone the packages into your catkin workspace and compile

    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)

    ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/westonrobot/scout_ros.git
    $ git clone https://github.com/cyberbotics/webots_ros.git
    $ git clone https://github.com/RoboSense-LiDAR/ros_rslidar.git
    $ cd ..
    $ catkin_make
    ```

3. Launch ROS nodes
 
* Start the base node 

    ```
    $ roslaunch scout_bringup scout_minimal.launch
    ```

    or (if you're using a serial port)
        
    ```
    $ roslaunch scout_bringup scout_minimal_uart.launch
    ```

* Start the keyboard tele-op node

    ```
    $ roslaunch scout_bringup scout_teleop_keyboard.launch
    ```

    **SAFETY PRECAUSION**: 

    The default command values of the keyboard teleop node are high, make sure you decrease the speed commands before starting to control the robot with your keyboard! Have your remote controller ready to take over the control whenever necessary. 

* Navigation with the robot

    Connect to the robot base and start onboard sensors

    ```
    $ roslaunch scout_bringup scout_nav_base.launch
    ```
    
    Or if you're using the simulator

    ```
    $ roslaunch scout_bringup scout_sim_nav_base.launch
    ```

    Then run the mapping or navigation launch file

    ```
    $ roslaunch scout_navigation scout_mapping_demo.launch
    ```

    ```
    $ roslaunch scout_navigation scout_navigation_demo.launch
    ```

