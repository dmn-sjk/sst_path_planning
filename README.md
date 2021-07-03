# Stable Sparse RRT (SST)
Implementation of SST path planning algorithm based on article: 

Li, Y., Littlefield, Z., & Bekris, K. E. (2016). Asymptotically optimal sampling-based kinodynamic planning. *The International Journal of Robotics Research*, 35(5), 528-564.

Car's kinematic model is used.

![visualization](https://user-images.githubusercontent.com/71564608/124363491-ce76b780-dc3b-11eb-9edf-bea447a9db9e.png)

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

## Run simulated SST path planning:
Configuration in simulation.launch file
```bash
roslaunch sst simulation.launch
```

## Check car kinematic model and control it with keyboard:
```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
roslaunch sst car_simulation.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```