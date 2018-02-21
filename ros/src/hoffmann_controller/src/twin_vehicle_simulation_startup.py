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
		rospy.Subscriber('/catvehicle2/front_laser_points', LaserScan, self.scanCallback)
		self.vel_pub = rospy.Publisher('/catvehicle2/cmd_vel', Twist, queue_size=10)
		
		self.reset_sim_pub1 = rospy.Publisher('/catvehicle1/reset_sim', String, queue_size=10)
		self.reset_sim_pub2 = rospy.Publisher('/catvehicle2/reset_sim', String, queue_size=10)

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
		pose1 = Pose()
		pose1.position.x = 0.0
		pose1.position.y = 0.0
		pose1.position.z = -0.30
		pose1.orientation.x = 0.0
		pose1.orientation.y = 0.0
		pose1.orientation.z = 0.0
		pose1.orientation.w = 0.0
		# set initial model state
		model_state1 = ModelState()
		model_state1.model_name = "catvehicle1"
		model_state1.pose = pose1
		self.set_model_state_client(model_state1)

		pose2 = pose1
		pose2.position.x = -20.0
		pose2.position.z = -0.30
		model_state2 = ModelState()
		model_state2.model_name = "catvehicle2"
		model_state2.pose = pose2
		self.set_model_state_client(model_state2)

		msg = String()
		msg.data = "123"
		self.reset_sim_pub1.publish(msg)
		self.reset_sim_pub2.publish(msg)

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
		while now - begin < 10000:
			now = rospy.Time.now().to_sec()
			rospy.loginfo("I am doing something ...")
			# self.vel_pub.publish(vel_msg)
			self.detect_collision(self.scan_ranges)
			if self.collision:
				self.collision = False
				rospy.logwarn("Collision!!!")
				self.reset()		

	def detect_collision(self,scan_ranges):
		min_range = 1.6
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
