# Autoware manual control

## Preparation

Clone this repository and load all submodules.

```
git clone https://github.com/NEWSLabNTU/F1EIGHT.git
cd F1EIGHT
git submodule update --init --recursive
```


Pull docker image and run in docker

```
# ubuntu 22.04 with humble installed
docker pull habby1012/humble22.04   
docker run --privileged -it --mount type=bind,source=/dev,target=/dev -v ~/F1EIGHT:/autoware_manual_control_ws --network=host habby1012/humble22.04
```

## Build

Setup ROS environment whenever you start a new shell.

```
source /opt/ros/humble/setup.bash
```


Build the workspace using `colcon`.

```
cd autoware_manual_control_ws
colcon build
```

## Usage
Source autoware_manual_control environment
```
source install/setup.bash
```
Use following command to send control message 
```
ros2 run autoware_manual_control keyboard_control
```
Use following command to control the car
```
ros2 run esc_control esc_control
```
