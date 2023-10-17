# ROS Installation and Setup Instructions

From http://wiki.ros.org/noetic/Installation/Ubuntu

## 1. Install ROS

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
```

## 2. Installs ROS Base package
```bash
sudo apt-get install ros-noetic-ros-base
```

## 3. Updates ROS dependencies
```bash
sudo rosdep init
rosdep update
```

## 4. Source the ROS setup script to setup the environment variables based on ROS Distro
```bash
source /opt/ros/noetic/setup.bash
```

## 5. Install additional packages to build ROS workspaces
```bash
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
pip3 install catkin_pkg
```

## 6. Check ROS was installed correctly and all environment variables are set
```bash
printenv | grep ROS
```

## 7. Ensure ROS_PACKAGE_PATH is set correctly
```bash
echo $ROS_PACKAGE_PATH
```

## 8. (Optional) Install ROS Tutorials
```bash
sudo apt-get install ros-noetic-ros-tutorials
```

## 9. Start roscore
```bash
roscore
```

# Daily Usage
```bash
source /opt/ros/noetic/setup.bash
roscore
```
