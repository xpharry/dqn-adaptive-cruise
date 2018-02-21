#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class Simulation:

	def __init__(self):
		# Launch the simulation with the given launchfile name
		# gazebo_env.GazeboEnv.__init__(self, "GazeboCartPole_v0.launch")

		self.scan_ranges = []
		self.collision = False
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', EmptySrv)

		# the callback method is called
		rospy.Subscriber('/catvehicle/front_laser_points', LaserScan, self.scanCallback)
		self.vel_pub = rospy.Publisher('/catvehicle/cmd_vel', Twist, queue_size=10)
		
		self.reset_sim_pub = rospy.Publisher('/catvehicle/reset_sim', String, queue_size=10)

	def scanCallback(self,data):
		self.scan_ranges = data.ranges

	def pause_physics_client(self):
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			pause_physics = rospy.ServiceProxy('/gazebo/pause_physics', EmptySrv)
			pause_physics()
		except rospy.ServiceException, e:
			print "Service \'/gazebo/pause_physics\' call failed: %s"%e

	def unpause_physics_client(self):
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
			unpause_physics = rospy.ServiceProxy('/gazebo/unpause_physics', EmptySrv)
			unpause_physics()
		except rospy.ServiceException, e:
			print "Service \'/gazebo/unpause_physics\' call failed: %s"%e

	def set_model_state_client(self, model_state_msg):
		rospy.wait_for_service('/gazebo/set_model_state')
		try:
			set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
			ret = set_model_state(model_state_msg)
			print ret.status_message
		except rospy.ServiceException, e:
			print "Service \'set_model_state\' call failed: %s"%e

	def reset(self):
		# # Resets the state of the environment and returns an initial observation.
		# rospy.wait_for_service('/gazebo/reset_world')
		# try:
		# 	#reset_proxy.call()
		# 	self.reset_proxy()
		# except (rospy.ServiceException) as e:
		# 	print ("/gazebo/reset_world service call failed")

		sim = rospy.get_param('/use_sim_time')
		if sim is True:
			rospy.loginfo('Waiting until simulated robot is prepared for the task...')
			# rospy.wait_for_message('/sim_robot_init', EmptyMsg)

		rospy.loginfo("Pausing physics")
		self.pause_physics_client()

		rospy.loginfo("Reset vehicle")

		# define initial pose for later use
		pose = Pose()
		pose.position.x = 0.0
		pose.position.y = 0.0
		pose.position.z = 0.0
		pose.orientation.x = 0.0
		pose.orientation.y = 0.0
		pose.orientation.z = 0.0
		pose.orientation.w = 0.0

		# set initial model state
		model_state = ModelState()
		model_state.model_name = "catvehicle"
		model_state.pose = pose

		self.set_model_state_client(model_state)

		msg = String()
		msg.data = "123"
		self.reset_sim_pub.publish(msg)

		rospy.loginfo("Unpausing physics")
		self.unpause_physics_client()

	def run(self):
		rospy.loginfo("Do something here.")

		vel_msg = Twist()
		vel_msg.linear.x = 2.0
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0
		
		one_sec = rospy.Duration(1)
		begin = rospy.Time.now().to_sec()
		now = rospy.Time.now().to_sec()
		while now - begin < 10:
			now = rospy.Time.now().to_sec()
			rospy.loginfo("I am doing something ...")
			# self.vel_pub.publish(vel_msg)
			self.detect_collision(self.scan_ranges)
			if self.collision:
				self.collision = False
				rospy.logwarn("Collision!!!")
		self.reset()		

	def detect_collision(self,scan_ranges):
		min_range = 5 #0.2
		for i in range(len(scan_ranges)):
			if (min_range > scan_ranges[i] > 0):
				self.collision = True
		return

def main():
	rospy.init_node('simulation_startup')
	sim = Simulation()
	rate = rospy.Rate(20) # run at 20Hz
	while not rospy.is_shutdown():
		sim.run()
		rate.sleep()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
