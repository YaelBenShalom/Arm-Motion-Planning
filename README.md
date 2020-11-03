# MECH_ENG_495 - Homework 3
GitHub repository - `https://github.com/ME495-EmbeddedSystems/homework-3-YaelBenShalom`


## Overview

This repository contains two packages: `arm_move` and `deff_drive`.
- `arm_move` package used the MoveIt Python API to create a planning scene and perform some path planning tasks The software plans a path to an object, without hitting an obsticle (a Realsense box), and if such path exists, the arm moves to the objects, pick it up and place it elswhere.

- `deff_drive` package uses Xacro files of a differential-drive robot and simulate it in Gazebo and controlled it using ROS. The robot is be able to follow a rectangular path or flip over and continue driving.