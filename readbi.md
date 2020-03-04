ssh weston@10.7.5.100
sudo systemctl start sensorhub.service

ssh weston@10.7.5.88
roslaunch scout_bringup scout_nav_base.launch
roslaunch scout_navigation scout_navigation_demo.launch
rosrun scout_ultrasonic ultra.py
roslaunch twist_mux.launch
