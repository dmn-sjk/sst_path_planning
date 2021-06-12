# sst_path_planning

## Build:
```bash
mkdir -p catkin_ws/src

cd ~/catkin_ws/src

git clone https://github.com/dmn-sjk/sst_path_planning.git sst

cd ~/catkin_ws

catkin_make

source devel/setup.bash

chmod +x ~/catkin_ws/src/sst/scripts/*.py
```

## Run car simulation:
```bash
roslaunch sst car_simulation.launch
```

## Control a car with keyboard:
```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Plan and execute path on map:
Configuration in simulation.launch file
```bash
roslaunch sst simulation.launch
```
