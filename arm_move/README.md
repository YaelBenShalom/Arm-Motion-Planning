# MECH_ENG_495 - Homework 3, Part 2
GitHub repository - `https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom`


## Overview

In this assignment I used the MoveIt Python API to create a planning scene and perform some path planning tasks (using `arm_move` package).
The software plans a path to an object, without hitting an obsticle (a Realsense box), and if such path exists, the arm moves to the objects, pick it up and place it elswhere
If the software fails to plan a path, it uses MoveItErrorCode to interpret the failure reason.

## Usage and Configuration instructions

1. To launch the arm path-planning package with the real robot and on RVIZ simulation, run `roslaunch arm_move arm.launch`.

    ![real robot - arm_move node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/arm_move/GIFs/arm_move.gif)

2. To launch the arm path-planning package without the real robot (only on RVIZ), add `use_fake:=True use_actual:=False`

    ![fake robot (rviz simulation) - arm_move node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/arm_move/GIFs/arm_move_rviz.gif)

3. To reset the scene (Taking a position and orientation of the realsense box and creates/moves the Realsense Box to the appropriate location), call the service `/px100/reset` (`rosservice call /px100/reset`). The service clears the waypoints recording (by default) and move the arm to it's home position.

    - To keep the waypoints recordings whenever the `/px100/reset` service is called, add `clear_waypoints:=False` when launching the package.

4. To follow the waypoints (in sequence), call the service `/px100/follow` (`rosservice call /px100/reset`). The service calls the `step` service and moves the arm to each waypoint. If there is no available path, the `step` service returns `MoveItErrorCode` and stop moving the arm (for success path planning, the service returns *MoveItErrorCode = 1*). The cycle of waypoints run once by default.

    - To make the waypoint cycle repeatedly when calling the `/px100/follow` service, add `run_once:=False` when launching the package.

5. To test the package using the `arm.test` launchfile, run `catkin_make run_tests` from the root of workspace. The test file calls the `reset` service and check the MoveItErrorCode return from the `step` service when trying to move the arm to impossible point.