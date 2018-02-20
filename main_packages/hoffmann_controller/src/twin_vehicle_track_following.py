#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from geometry_msgs.msg import Twist, Pose, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import sys
import math

UPDATE_RATE = 20.0  # choose the update rate for steering controller
K_PSI= 0.7  # control gains for steering

# dynamic limitations:  
MAX_SPEED = 5.0  # m/sec; tune this
MAX_OMEGA = 1.0  # rad/sec; tune this

class Simulation:

    def __init__(self):
        # Launch the simulation with the given launchfile name
        # gazebo_env.GazeboEnv.__init__(self, "GazeboCartPole_v0.launch")

        self.path = []
        self.pose = Pose()

        self.des_x = 0
        self.des_y = 0
        self.des_quat = 0
        self.des_psi = 0
        self.des_speed = 5

        self.x = 0
        self.y = 0
        self.quat = 0
        self.psi = 0
        self.speed = 0

        self.scan_ranges = []
        self.collision = False
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', EmptySrv)

        self.vel_pub = None
        self.reset_sim_pub1 = None
        self.reset_sim_pub2 = None

        self.path = self.generate_track_path()
        self.init_subscribers()
        self.init_publishers()

    # member helper function to set up subscribers;
    def init_subscribers(self):
        rospy.loginfo("Initializing Subscribers: gazebo state and desState")
        rospy.Subscriber('/catvehicle2/odom', Odometry, self.odomCallback)
        rospy.Subscriber('/catvehicle2/front_laser_points', LaserScan, self.scanCallback)

    def init_publishers(self):
        self.vel_pub = rospy.Publisher('/catvehicle2/cmd_vel', Twist, queue_size=10)
        self.reset_sim_pub1 = rospy.Publisher('/catvehicle1/reset_sim', String, queue_size=10)
        self.reset_sim_pub2 = rospy.Publisher('/catvehicle2/reset_sim', String, queue_size=10)

    def odomCallback(self, data):
        self.pose = data.pose.pose
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.quat = data.pose.pose.orientation
        self.psi = self.quat2phi(self.quat)
        self.speed = data.twist.twist.linear.x        

    def scanCallback(self, data):
        self.scan_ranges = data.ranges

    def min_dang(self, dang):
        while dang > math.pi:
            dang -= 2.0 * math.pi
        while dang < -math.pi:
            dang += 2.0 * math.pi
        return dang

    # saturation function, values -1 to 1
    def sat(self, x):
        if x > 1.0:
            return 1.0
        if x < -1.0:
            return -1.0
        return x

    def sign(self, x):
        if x > 1.0:
            return 1.0
        if x < -1.0:
            return -1.0
        return x

    def quat2phi(self, quat):
        quat_z = quat.z
        quat_w = quat.w
        phi = 2.0 * math.atan2(quat_z, quat_w)
        return phi

    def phi2quat(self, phi):
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(phi / 2.0)
        quaternion.w = math.cos(phi / 2.0)
        return quaternion

    def generate_track_path(self):
        ra = 36.6
        nedges = 301

        path = []
        pose = Pose()

        # straight line 1
        for i in range(nedges):
            # point i
            pose.position.x = i * 2 * ra / nedges - ra
            pose.position.y = 0.0
            pose.position.z = 0.0

            theta = 0.0
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        # half circle 1
        phi0 = -math.pi / 2
        for i in range(nedges / 2):
            # point i
            phi = self.min_dang(phi0 + 2.0 * math.pi / nedges * i)
            pose.position.x = ra * math.cos(phi) + ra
            pose.position.y = ra * math.sin(phi) + ra
            pose.position.z = 0.0

            theta = self.min_dang(phi + math.pi / 2.0)
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        # straight line 2
        for i in range(nedges):
            # point i
            pose.position.x = - i * 2 * ra / nedges  + ra
            pose.position.y = 2.0 * ra
            pose.position.z = 0.0

            theta = math.pi
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        # half circle 2
        phi0 = -math.pi / 2
        for i in range(nedges / 2, nedges):
            # point i
            phi = self.min_dang(phi0 + 2.0 * math.pi / nedges * i)
            pose.position.x = ra * math.cos(phi) + ra
            pose.position.y = ra * math.sin(phi) + ra
            pose.position.z = 0.0

            theta = self.min_dang(phi + math.pi / 2.0)
            quat = self.phi2quat(theta)
            pose.orientation = quat

            path.append(pose)

        return path

    def compute_dist(self, a, b):
        x1 = a.position.x
        y1 = a.position.y
        x2 = b.position.x
        y2 = b.position.y
        return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

    def find_index(self):
        min_dist = sys.float_info.max
        for i, ele in enumerate(self.path):
            dist = self.compute_dist(self.pose, self.path[i])
            if dist < min_dist:
                min_idx = i
                min_dist = dist
        return min_idx

    def calculate_track_dist(self, p1, p2):
        unit_dist = self.compute_dist(path[0], path[1])
        index1 = self.find_index(p1)
        index2 = self.find_index(p2)
        num_units = index1 - index2
        num_units = num_units if num_units > 0 else num_units + len(path)
        track_dist = num_units * unit_dist
        return track_dist

    def speed_cmd_fnc(self, des_speed, dist_err):
        des_speed = 0 if dist_err > 4 else des_speed
        if des_speed > self.speed:
            cmd_speed = self.speed + 0.1
        elif des_speed < self.speed:
            cmd_speed = self.speed - 0.1
        return cmd_speed

    def omega_cmd_fnc(self, heading_err, offset_err, des_speed):
        omega_cmd = heading_err + math.atan2(K_PSI * offset_err, des_speed);
        return -omega_cmd

   # HERE IS THE STEERING ALGORITHM
    def hoffmann_steering(self):
        index = self.find_index()

        self.des_x = self.path[index].position.x
        self.des_y = self.path[index].position.y
        self.des_quat = self.path[index].orientation
        self.des_psi = self.min_dang(self.quat2phi(self.des_quat))

        dx = self.des_x - self.x  # x-error relative to desired path
        dy = self.des_y - self.y  # y-error
        tx = math.cos(self.des_psi)  # [tx,ty] is tangent of desired path
        ty = math.sin(self.des_psi) 
        nx = ty  # components [nx, ny] of normal to path, points to left of desired heading
        ny = -tx

        signnn = self.sign(dx*nx + dy*ny)
        offset = math.sqrt(dx*dx + dy*dy)
        lateral_err = signnn * offset
        trip_dist_err = dx*tx + dy*ty  # progress error: if positive, then we are ahead of schedule    
        # heading error: if positive, should rotate -omega to align with desired heading
        heading_err = self.min_dang(self.psi - self.des_psi)
        
        cmd_speed = self.speed_cmd_fnc(self.des_speed, lateral_err)
        cmd_omega = self.omega_cmd_fnc(heading_err, lateral_err, cmd_speed)

        twist_msg = Twist()
        twist_msg.linear.x = cmd_speed
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = cmd_omega
        self.vel_pub.publish(twist_msg)

        return cmd_omega

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
        #   #reset_proxy.call()
        #   self.reset_proxy()
        # except (rospy.ServiceException) as e:
        #   print ("/gazebo/reset_world service call failed")

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

    def detect_collision(self,scan_ranges):
        min_range = 1.6
        for i in range(len(scan_ranges)):
            if (min_range > scan_ranges[i] > 0):
                self.collision = True
        return

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

def main():
    rospy.init_node('track_following')
    sim = Simulation()
    rate = rospy.Rate(20) # run at 20Hz
    while not rospy.is_shutdown():
        sim.hoffmann_steering()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
