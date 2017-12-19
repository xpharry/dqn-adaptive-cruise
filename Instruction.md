## Example use 1

1) roslaunch catvehicle two_catvehicles_skidpan.launch

	* nothings happen here
	* rostopic list
	* rosnode list

2) gzclient

	show two vehicles in 3D

3) rosrun rviz rviz

4) rosrun mobot_gazebo_state mobot_gazebo_state

5) rosrun mobot_pub_des_state open_loop_controller

6) rosrun mobot_pub_des_state mobot_pub_des_state

7) rosrun mobot_pub_des_state pub_des_state_path_client

## Example use 2

1) roslaunch catvehicle catvehicle_skidpan.launch

	* nothings happen here
	* rostopic list
	* rosnode list

2) gzclient

	show two vehicles in 3D

3) rosrun rviz rviz

4) rosrun mobot_gazebo_state mobot_gazebo_state

5) rosrun lin_steering lin_steering_wrt_odom

6) rosrun mobot_pub_des_state mobot_pub_des_state

7) rosrun mobot_pub_des_state pub_des_state_path_client

