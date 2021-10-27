Scout Mini ROS Packages
=======================

Overview
---------
ROS packages for Scout Mini

[Scout Mini Tutorial](https://docs.roas.co.kr/scout_mini.html)

Installation
------------

```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/scout_mini_ros.git
cd ~/catkin_ws/src/scout_mini_ros/scout_mini_base/lib/
sudo dpkg -i ros-melodic-scout-mini-lib-* (your PC architecture)

cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

Start
-----

```
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_mini_base base.launch
```

Upstart
-------

```
rosrun scout_mini_lib install_upstart -r scout_mini
sudo reboot
```
