# scout_mini_ros
[![Licence](https://img.shields.io/badge/License-BSD--3-green.svg)](https://opensource.org/license/bsd-3-clause/)
[![ubuntu20](https://img.shields.io/badge/-UBUNTU_20.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/focal/)
[![noetic](https://img.shields.io/badge/-NOETIC-blue?style=flat-square&logo=ros)](https://wiki.ros.org/noetic)

## Overview
ROS packages for Scout Mini

[Specification](https://roas.co.kr/scout-mini/)<br>
[Manual](https://docs.roas.co.kr/scout_mini.html)

## Installation
```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/scout_mini_ros.git

cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

## Usage
```
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_mini_base base.launch
```
