## Adaptive Cruise Control and Lane Control using Deep Q-Learning

## Overview

The work aims at design of a learning based autonomous driving system mainly on adaptive cruise control (ACC) and lane control (LC). In this study a simulated highway environment is created for the vehicles to capture online training data. A Deep Q-Learning algorithm is proposed to train two agents respectively, an ACC agent and an integrated agent (ACC and LC). The results show behavioral adaptation with an ACC in terms of the speed of the preceding vehicle and emergence brake and LC in terms of two lanes. The learning based system has a nice adaption in different scenarios and can be reinforced by continuous training.

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



