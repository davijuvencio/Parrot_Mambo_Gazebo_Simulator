# eRobotica_Mambo_Gazebo
Um modelo ROS/Gazebo do Mambo parrot adaptado de RotorS simulator por Davi Juvêncio (davi.j.g.sousa@ee.ufcg.edu.br).

## Pré requisito
- Ubuntu 20.04
- ROS Noetic 
## Installation

```bash
mkdir -p eRobotica_Mambo_Gazebo/src && cd eRobotica_Mambo_Gazebo/src

sudo apt install build-essential python3-rosdep python3-catkin-tools
sudo apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev
sudo apt install ros-noetic-mavros ros-noetic-octomap-ros 

git clone https://github.com/ethz-asl/mav_comm
git clone https://github.com/davijuvencio/eRobotica_Mambo_Gazebo
git clone https://github.com/ros-drivers/joystick_drivers

cd ..
source /opt/ros/noetic/setup.bash
catkin_make
```

##### Simular
```bash
roslaunch rotors_gazebo Mambo.launch
```
