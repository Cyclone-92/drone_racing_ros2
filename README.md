# <div align="center">`drone_racing_ros2`</div>
# <div align="center">`University of Turku: Aerial Robotics Group 5 - Common Project (DTEK2084)`</div>
## Running a Tello simulation in [Gazebo](http://gazebosim.org/)

`tello_gazebo` consists of several components:
* `TelloPlugin` simulates a drone, handling takeoff, landing, and very simple flight dynamics.
* `markers` contains Gazebo models for fiducial markers.
* `fiducial.world` is a simple world with a bunch of fiducial markers.
* `inject_entity.py` is a script that will read a URDF (ROS) or SDF (Gazebo) file and spawn a model in a running instance of Gazebo.
* `drone_controller.py` is the script to control the drone autonomously.
* The built-in camera plugin is used to emulate the Gazebo forward-facing camera.

## Installation
#### Install ROS2 Galactic
    https://docs.ros.org/ with the `ros-galactic-desktop` option.
#### Make sure you have Gazebo 
    sudo apt install gazebo11 libgazebo11 libgazebo11-dev
#### Add the following
    sudo apt install libasio-dev
    sudo apt install ros-galactic-cv-bridge ros-galactic-camera-calibration-parsers 
    sudo apt install libignition-rendering3 
    pip3 install transformations

#### Build this package
    mkdir -p ~/drone_racing_ros2_ws/src
    cd ~/drone_racing_ros2_ws/src
    git clone https://github.com/Cyclone-92/drone_racing_ros2.git
    cd ..
    source /opt/ros/galactic/setup.bash
    colcon build
    
#### Run a teleop simulation

    cd ~/drone_racing_ros2_ws
    source install/setup.bash
    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh
    ros2 launch tello_gazebo team_05_launch.py
    
You will see a single drone in a world with 13 different gates with a Euro pallet and a stop sign.

If you run into the **No namespace found** error, re-set `GAZEBO_MODEL_PATH`:

    export GAZEBO_MODEL_PATH=${PWD}/install/tello_gazebo/share/tello_gazebo/models
    source /usr/share/gazebo/setup.sh

## Changing the sequence.
if you need to change the sequence of the gates, then you need to modify the sequence tuples in the drone controller.py. each tuple represents a gate.

tuple = (shape,color)

shape == > circle : 0 square :1 

color == > red : 0    green  :1 orange : 2 

    self.control_seqence = [(1,0),(0,1),(1,2),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(1,1),(0,3)]
    
## Authors:

     - Prashan Herath [prashan.r.herathmudiyanselage@utu.fi]
     - Julian C. Paez P. [julian.c.paezpineros@utu.fi]
     - Michalis Iona [michalis.l.iona@utu.fi]
     
University of Turku TIERS lab group.
