# Arm Motion Planning


## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
    - [Dependencies installation instructions](#dependencies-installation-instructions)
- [Usage and Configuration Instructions](#usage-and-configuration-instructions)


## Overview

In this package I used the MoveIt Python API to create a planning scene and perform some path planning tasks.<br>
The software plans a path to an object, without hitting an obstacle (a Realsense box), and if such path exists, the arm moves to the objects, pick it up and place it elsewhere.<br>
If the software fails to plan a path, it uses MoveItErrorCode to interpret the failure reason.


## Getting Started
Create a workspace, clone the repo, and build the workspace:
```
mkdir -p ws/src && cd ws/src
git clone https://github.com/YaelBenShalom/Arm-Motion-Planning.git
cd ../..
catkin_make
source devel/setup.bash 
```


### Dependencies Installation Instructions

1. Install the RealSense SDK:
    ```
    # Install intel's keys
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

    # Add the apt repository
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"

    # Update the package list
    sudo apt update

    # Install the packages
    sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg librealsense2-gl-dev ros-noetic-realsense2-camera ros-noetic-realsense2-description

    # Install the python wrapper
    pip3 install pyrealsense2 
    ```

2. Install the PincherX 100 Interbotix library:
    ```
    # Create the workspace
    mkdir -p ~/custom_ws/src
    cd custom_ws/src

    # Clone the repositories
    git clone https://github.com/Interbotix/interbotix_ros_core.git
    git clone https://github.com/Interbotix/interbotix_ros_manipulators.git -b noetic
    git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git

    # Remove some CATKIN_IGNORE files so we build these packages
    find . -name CATKIN_IGNORE | xargs rm

    # Install the udev rules for the arm
    sudo cp interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk/99-interbotix-udev.rules /etc/udev/rules.d
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    # Install remaining depenencies and build the workspace
    cd ~/custom_ws
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    ```


## Usage and Configuration instructions

1. To launch the arm path-planning package with the real robot and on RVIZ simulation, run `roslaunch arm_move arm.launch`.

    <p align="center">
        <img align="center" src="https://github.com/YaelBenShalom/Arm-Motion-Planning/blob/master/GIFs/arm_move.gif">
    </p>

2. To launch the arm path-planning package without the real robot (only on RVIZ), add `use_fake:=True use_actual:=False`

    <p align="center">
        <img align="center" src="https://github.com/YaelBenShalom/Arm-Motion-Planning/blob/master/GIFs/arm_move_rviz.gif">
    </p>

3. To reset the scene (Taking a position and orientation of the Realsense box and creates/moves the Realsense Box to the appropriate location), call the service `/px100/reset` (`rosservice call /px100/reset`). The service clears the waypoints recording (by default) and move the arm to it's home position.

    - To keep the waypoints recordings whenever the `/px100/reset` service is called, add `clear_waypoints:=False` when launching the package.

4. To follow the waypoints (in sequence), call the service `/px100/follow` (`rosservice call /px100/reset`). The service calls the `step` service and moves the arm to each waypoint. If there is no available path, the `step` service returns `MoveItErrorCode` and stop moving the arm (for success path planning, the service returns *MoveItErrorCode = 1*). The cycle of waypoints run once by default.

    - To make the waypoint cycle repeatedly when calling the `/px100/follow` service, add `run_once:=False` when launching the package.

5. To test the package using the `arm.test` launchfile, run `catkin_make run_tests` from the root of the workspace. The test file calls the `reset` and `step` services and checks the MoveItErrorCode returns from the `step` service, when trying to move the arm to impossible point. If the test result is `SUCCESS` - it means that the `step` services returned *MoveItErrorCode = -1* (planning failed) as expected.
