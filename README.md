# Auto Vehicle

## Overview

This work ...

## Instruction

## Developing Log

**9/27:**

* Duplicate [mybot package](https://github.com/xpharry/making_my_robot_in_gazebo.git) into the repo.

**9/29:**

* Delete mybot from the master branch.
* Create three new branches for the mobot, P3-dx and potantial truck.

**10/3:**

* Working in Branch *mybot*.
* Modeling roads in gazebo.

**10/4:**

* Begin testing open-loop control.

**10/10:**

* Add in dummy open-loop control.
* Add in lidar alarm node.
* Add in service client commanders.
* Note "mybot" is not dynamically controlled well. Considering change to a more accurate robot.

**10/11:**

* Switch mybot to mobot and create a new branch "mobot". Open loop control is proved more stable.

**10/20:**

* Add in "mobot_pub_des_state" and "traj_builder". Compiled!

**10/24:**

* Catvehicle Testbed is tested and promising.
* Create a new branch "catvehicle".
* To do:
  * A circle loop motion with mobot.
  * Enable GPS on catvehicle.
  * Combile these two.
  * Create launch files and reports.
  
**12/9:**

* Hoffmann Controller in Simulink is tested.

**12/15:**

* Wrote a Hoffmann Controller in cpp and tested well.

**12/16:**

* Added in the second Catvehicle and made the both run together in the same trajectory.

**12/20:**

* Added in adaptive cruise control.

**12/21:**

* Reviewed my previous BehaviorLearning project and set up deep learning env.

**1/9/2018:**

* Trained a braker model with CNN.
* Applied the braker on the simulation.

**1/21/2018:**

* Replace the hard coded map by a csv file and display it in rviz.
* Found how to reset Gazebo: `rosservice call /gazebo/reset_world`
* In C++, an example to set initial gazebo state is like:

    `set_model_state_client =
	  n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");`

**1/23/2018:**

* Realized simulation re-initialization and tested on twin vehicle case.
* Added in C++11 support for Catvehicle. Simply add in a line `add_compile_options(-std=c++11)` in CMakeLists.txt.

**2/14/2018:**

* Trained a dqn model for vehicle on the track.

**2/21/2018:**

* Working one path planning.
* Reformat folders.

## Resources

1. Dr. Kostas Alexis, [Introduction to Aerial Robotics](http://www.kostasalexis.com/introduction-to-aerial-robotics.html)

## Reference

1. [Tutorials of CAT VEHICLE TESTBED](https://cps-vo.org/node/31792)


