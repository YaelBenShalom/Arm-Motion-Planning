# MECH_ENG_495 - Homework 3
GitHub repository - `https://github.com/YaelBenShalom/Gazebo-Differential-Drive-and-Arm-Motion-Planning`


## Overview

This repository contains two packages: `arm_move` and `deff_drive`.
1. `arm_move` package used the MoveIt Python API to create a planning scene and perform some path planning tasks The software plans a path to an object, without hitting an obstacle (a Realsense box), and if such path exists, the arm moves to the objects, pick it up and place it elsewhere.

    ![real robot - arm_move node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/arm_move/GIFs/arm_move.gif)

    Check out the package to learn more!

2. `diff_drive` package uses Xacro files of a differential-drive robot and simulate it in Gazebo and controlled it using ROS. The robot is be able to follow a rectangular path or flip over and continue driving.

    ![robot simulation in Gazebo - follow_rect node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/diff_drive/GIFs/follow_rect.gif)

    ![robot simulation in Gazebo - flip_over node](https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom/blob/master/diff_drive/GIFs/flip_over.gif)

    Check out the package to learn more!
