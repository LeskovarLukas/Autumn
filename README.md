<h1 align="center">Autonomous universal Mapping and Navigation</h1>

## How to Use Autumn

### 1. Prerequisites

* ROS Melodic
* Ubuntu 18.04

### 2. Dependencies

```
sudo apt install python-wstool python-catkin-tools 
sudo apt install ros-melodic-pcl-ros ros-melodic-pcl-conversions
```

### 3. Installation

```shell
# Create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clone Autumn
cd ~/catkin_ws/src/
git clone https://github.com/F-WuTS/Autumn.git
wstool init . ./Autumn/autumn.rosinstall

# Compile
catkin build
```
