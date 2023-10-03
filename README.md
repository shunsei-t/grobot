# Enviroment
```
ros noetic
```

# Dependencies
## rplidar
```
sudo apt install ros-noetic-rplidar
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
roslaunch grobot robot gmapping.launch
```