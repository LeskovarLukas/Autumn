<h1 align="center">Autonomous universal Mapping and Navigation</h1>

## How to Use

### 1. Prerequisites

* ROS Melodic
* Ubuntu 18.04

### 2. Dependencies

```
sudo apt install python-wstool python-catkin-tools 
sudo apt install ros-melodic-pcl-ros ros-melodic-pcl-conversions
```

Fnstall the [DJI-Onboard-SDK](https://github.com/dji-sdk/Onboard-SDK) 
(for further instructions view the [Documentation](https://developer.dji.com/onboard-sdk/documentation/introduction/how-to-use-OSDK.html)
or [ROS-Wiki](http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started))


### 3. Installation

```shell
# Create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clone Autumn
cd ~/catkin_ws/src/
git clone git@github.com:F-WuTS/Autumn.git
wstool init . ./Autumn/autumn.rosinstall

# Configuration
copy the contents of `sdk_launch.txt` into `~/catkin_ws/src/Onboard-SDK-ROS/dji_sdk/launch/sdk.launch`

# Compile
catkin build
```
