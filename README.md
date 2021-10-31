# Gazebo Differential Drive and Arm Motion Planning


## Table of Contents

- [Description](#description)
- [Getting Started](#getting-started)
- [Usage and Configuration Instructions](#usage-and-configuration-instructions)


## Description
This repository contains two packages: `arm_move` and `deff_drive`:<br>
1. `arm_move` - In this assignment I used the MoveIt Python API to create a planning scene and perform some path planning tasks (using arm_move package). The software plans a path to an object, without hitting obstacles, and if such path exists, the arm moves to the objects, pick it up and place it elsewhere If the software fails to plan a path, it uses MoveItErrorCode to interpret the failure reason.


2. `diff_drive` - In this assignment I created Xacro files of a differential-drive robot, simulate it in Gazebo and control it using ROS (using diff_drive package).
    1. The robot moves through a world filled with Jersey Barriers and trash.
    2. The robot is be able to follow a rectangular path.
    3. The robot is be able to flip over and continue driving.


## Getting Started

Create a workspace, clone the repo, and build the workspace:
```
mkdir -p ws/src && cd ws/src
git clone https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning.git
cd ../..
catkin_make
source devel/setup.bash 
```


## Usage and Configuration Instructions

This repository contains two packages: `arm_move` and `deff_drive`.
1. `arm_move` package used the MoveIt Python API to create a planning scene and perform some path planning tasks. The software plans a path to an object, without hitting obstacles, and if such path exists, the arm moves to the objects, pick it up and place it elsewhere.

    1. To launch the arm path-planning package with the real robot and on RVIZ simulation, run `roslaunch arm_move arm.launch`.

        <p align="center">
            <img align="center" src="https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning/blob/master/arm_move/GIFs/arm_move.gif">
        </p>

    2. To launch the arm path-planning package without the real robot (only on RVIZ), add `use_fake:=True use_actual:=False`

        <p align="center">
            <img align="center" src="https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning/blob/master/arm_move/GIFs/arm_move_rviz.gif">
        </p>

    3. To reset the scene (Taking a position and orientation of the Realsense box and creates/moves the Realsense Box to the appropriate location), call the service `/px100/reset` (`rosservice call /px100/reset`). The service clears the waypoints recording (by default) and move the arm to it's home position.

        - To keep the waypoints recordings whenever the `/px100/reset` service is called, add `clear_waypoints:=False` when launching the package.

    4. To follow the waypoints (in sequence), call the service `/px100/follow` (`rosservice call /px100/reset`). The service calls the `step` service and moves the arm to each waypoint. If there is no available path, the `step` service returns `MoveItErrorCode` and stop moving the arm (for success path planning, the service returns *MoveItErrorCode = 1*). The cycle of waypoints run once by default.

        - To make the waypoint cycle repeatedly when calling the `/px100/follow` service, add `run_once:=False` when launching the package.

    5. To test the package using the `arm.test` launchfile, run `catkin_make run_tests` from the root of the workspace. The test file calls the `reset` and `step` services and checks the MoveItErrorCode returns from the `step` service, when trying to move the arm to impossible point. If the test result is `SUCCESS` - it means that the `step` services returned *MoveItErrorCode = -1* (planning failed) as expected.

2. `diff_drive` package uses Xacro files of a differential-drive robot and simulate it in Gazebo and control it using ROS. The robot is be able to follow a rectangular path or flip over and continue driving.

    1. To launch the differential-drive robot in the `ddrive` world using Gazebo simulation, run `roslaunch diff_drive ddrive.launch`. The robot starts from position *(x,y) = (-3,-3)* in rest mode.

        - To make the robot follow a rectangular path, add `follow_rect:=True` to the roslaunch command.

        <p align="center">
            <img align="center" src="https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning/blob/master/diff_drive/GIFs/follow_rect.gif">
        </p>

        - To make the robot flip over and continue driving, add `flip_over:=True` to the roslaunch command.

        <p align="center">
            <img align="center" src="https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning/blob/master/diff_drive/GIFs/flip_over.gif">
        </p>

    2. To launch the differential-drive robot using RViz simulation, run `roslaunch diff_drive ddrive_rviz.launch`. The robot starts from position *(x,y) = (-3,-3)* in rest mode.

        <p align="center">
            <img align="center" src="https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning/blob/master/diff_drive/GIFs/follow_rect_rviz.gif">
        </p>



    Check out the package to learn more!
