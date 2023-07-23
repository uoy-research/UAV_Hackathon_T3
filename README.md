# UAV_Hackathon_T3
Team 3 for the UAV Hackathon: May 2023

## Installation Gazebo Classic:
```sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```


## Install ROS 2:
```sh
https://docs.ros.org/en/foxy/Installation.html
```

## Setuptools dependencies:
```sh
pip install --user -U empy pyros-genmsg setuptools
```

## DDS Client:
```sh
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build\
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib
```


## Gazebo Models:
```sh
Add the gazebo and models and worlds from this repo to
 /PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```
 ## Build ROS2 workspace:
 ```sh
cd /T3_ROS2_WS
colcon build
```
 

## For teleop:
 ```sh
(remember to source install/local_setup.bash in directory of workspace before running ROS2 commands)
Terminal 1: (from /PX4-Autopilot) make px4_sitl gazebo-classic_iris__mine
Terminal 2: MicroXRCEAgent udp4 -p 8888
Terminal 3: (from /T3_ROS2_WS) ros2 run keyboard_teleop keyboard_teleop_incremental 
Terminal 4: (from /T3_ROS2_WS) ros2 run  px4_ros_com control
```


## For control:
 ```sh
Publish twist messages to "cmd_vel"
ros2 run  px4_ros_com teleop
```


