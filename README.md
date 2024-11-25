# Autonomous UGV Digital Twin Simulation (AUDiTS)
This project uses the [Clearpath Robotics Simulator](https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/overview), ROS2 and Gazebo to simulate a Husky Unmanned Ground Vehicle (UGV) and test waypoint navigation, target tracking, and path following algorithms. Refer to [wiki](https://github.com/daylanc/AUDiTS/wiki#introduction) to modify simulation environment and additional information on running scripts.

## Dependencies
1. [Ubuntu 22.04](https://releases.ubuntu.com/jammy/)
2. [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
3. [Clearpath Desktop](https://github.com/clearpathrobotics/clearpath_desktop)
```
sudo apt install ros-humble-clearpath-desktop
```
4. Gazebo Ignition Fortress
```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```
5. [ros_gz_sim](https://github.com/gazebosim/ros_gz/tree/humble)
```
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update
sudo apt install ros-humble-ros-gz
```
6. [ign_ros2_control](https://github.com/ros-controls/gz_ros2_control/tree/humble)
```
sudo apt install ros-humble-ign-ros2-control
```

## Install
```
mkdir ~/clearpath_ws/src -p
cd ~/clearpath_ws/src
git clone https://github.com/daylanc/AUDiTS.git
git submodule update --init --recursive
cd ~/clearpath_ws
rosdep install -r --from-paths src -i -y
colcon build
```

## Setup
1. Robot.yaml file
```
mkdir ~/clearpath/
cp </path/to/robot.yaml> ~/clearpath/
source /opt/ros/humble/setup.bash
ros2 run clearpath_generator_common generate_bash -s ~/clearpath
source ~/clearpath/setup.bash
```
2. .bashrc file
- Open bashrc file
```
gedit ~/.bashrc
```
- Add to bottom of bashrc file
```
# ROS2 Dependencies
source /opt/ros/humble/setup.bash
source ~/clearpath_ws/install/setup.bash
source ~/clearpath/setup.bash
```

## Run Simulation
```
ros2 launch clearpath_gz simulation.launch.py rviz:=true world:=empty_world
```
- Parameters:
  - Launch RViz: ``` rviz:=true ```
  - Change starting coordinates: ``` x:=1.5 y:=2.7 yaw:=1.57 ```
  - Change world file: ``` world:=my_world ```
  - robot.yaml file path: ``` setup_path:=$HOME/setup/path/ ```
  
 
