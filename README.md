# Parrot Mambo Gazebo Simulator
Um modelo ROS/Gazebo do Mambo parrot adaptado de RotorS simulator por Davi Juvêncio (davijuvencio@gmail.com).

## Pré requisito
- Ubuntu 20.04
- ROS Noetic 
## Installation

```bash
mkdir -p Mambo_Gazebo/src && cd Mambo_Gazebo/src

sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros 

git clone https://github.com/ethz-asl/mav_comm
git clone https://github.com/davijuvencio/Parrot_Mambo_Gazebo_Simulator.git
git clone https://github.com/ros-drivers/joystick_drivers

cd ..
source /opt/ros/noetic/setup.bash
catkin_make
```

##### Simular
```bash
roslaunch rotors_gazebo Mambo.launch
```
