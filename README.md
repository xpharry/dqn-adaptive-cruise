# Auto Vehicle

## Overview

This work ...

## Installation

### Ubuntu 14.04 

- Install ROS Indigo
	
	[Installation Guide](http://wiki.ros.org/indigo/Installation/Ubuntu)

- Install Dependencies

	```
	sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers
	sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
	sudo apt-get install ros-indigo-velodyne
	```

### Ubuntu 16.04

- Install ROS Kinetic

	[Installation Guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)

- Install Dependencies

	```
	sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers
	sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control
	sudo apt-get install ros-kinetic-velodyne
	```

## Instruction

## Experiments

* Deep Q-Learning experiments on diverse scenarios.

	- Longitudinal Motion Control

		- A preceding vehicle and a trailing vehicle at constant speeds

		- A preceding vehicle and a trailing vehicle at erratic speeds

	- Longitudinal Motion Control

		- Two Lanes with 1-4 vehicles

		- Three Lanes with 1-6 vehicles

	- Longitudinal Motion and Latreral Motion Control

		- Three Lanes with 1-6 vehicles

* FCNN and CNN

**3/31/2018**

* Added in Cartpole as a simpler example to use dqn.
* Enabled online visualization and auto save of reward history.

## Resources

1. Dr. Kostas Alexis, [Introduction to Aerial Robotics](http://www.kostasalexis.com/introduction-to-aerial-robotics.html)

2. [Tutorials of CAT VEHICLE TESTBED](https://cps-vo.org/node/31792)


