# Enviroment
```
ros noetic
```

# Dependencies
## On robot
Install rplidar driver node
```
sudo apt install ros-noetic-rplidar
```
Install Dynamixel-SDK
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK
python3 -m pip install ./python
```
## On PC
Install emcl2
```
cd ~/your_ws/src
git clone  https://github.com/ryuichiueda/emcl2.git
cd ~/your_ws
catkin_make
source devel/setup.bash
```
Install navigation packages
```
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-turtlebot3
sudo apt install ros-noetic-turtlebot3-gazebo
```

# How to use
## Controled by PS4 controller
### On robot
```
roslaunch grobot robot_controller.launch
```
### On pc
```
roslaunch grobot joy_control.launch
```
## Mapping
```
roslaunch grobot gmapping.launch
```
## navigation
```
roslacunch grobot emcl2nav.launch
```