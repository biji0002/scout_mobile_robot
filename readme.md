#supported by Bi,Jianxin: bijianxin292430887@gmail.com

#upboard service
ssh weston@10.7.5.100
sudo systemctl start sensorhub.service
#if ping 10.7.5.100 fail, try unmount all serial ports, usb, LAN cables, RTC battery, etc.
#connect keyboard and monitor, restart the upboard, press [delete], if you are lucky you will see ubuntu shows, pb solved.
#if cannot see anything on monitor, do more times until you want to give up. probably it is truly broken who knows :( 
		

#robot sensing system
ssh weston@10.7.5.88
roslaunch scout_bringup scout_nav_base.launch

# navigation
roslaunch scout_navigation scout_navigation_demo.launch 


# ultrasonic sensing
rosrun scout_ultrasonic ultra.py
roslaunch twist_mux.launch


# ptz camera control
rosrun scout_base ptz_service_call_node
rosrun scout_base ptz_ctrl_publisher #use w,a,s,d to control the movement of PTZ camera.
