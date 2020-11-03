# MECH_ENG_495 - Homework 3, Part 1
GitHub repository - `https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom`


## Overview

In this assignment I created Xacro files of a differential-drive robot, simulate it in Gazebo and controlled it using ROS (using `diff_drive` package).
1. The robot moves through a world filled with Jersey Barriers and trash.
2. The robot is be able to follow a rectangular path.
3. The robot is be able to flip over and continue driving.



## Usage and Configuration instructions

1. To launch the differential-drive robot in the `ddrive` world using Gazebo simulation, run `roslaunch diff_drive ddrive.launch`. The robot starts from position *(x,y) = (-3,-3)* in rest mode.

    1. To make the robot follow a rectangular path, add `follow_rect:=True` to the roslaunch command.

    ![robot simulation in Gazebo - follow_rect node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/diff_drive/GIFs/follow_rect.gif)

    2. To make the robot flip over and continue driving, add `flip_over:=True` to the roslaunch command.

    ![robot simulation in Gazebo - flip_over node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/diff_drive/GIFs/flip_over.gif)

    3. To launch the differential-drive robot using RViz simulation, run `roslaunch diff_drive ddrive_rviz.launch`

    ![robot simulation in rviz - follow_rect node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/diff_drive/GIFs/follow_rect_rviz.gif)

