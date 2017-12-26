// hoffmann_controller.h header file //
// Peng Xu; Dec, 2017

#ifndef HOFFMANN_CONTROLLER_H_
#define HOFFMANN_CONTROLLER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>

const double UPDATE_RATE = 20.0; // choose the update rate for steering controller
const double K_PSI= 0.7; // control gains for steering

// dynamic limitations:  
const double MAX_SPEED = 5.0; // m/sec; tune this
const double MAX_OMEGA = 1.0; // rad/sec; tune this


// define a class, including a constructor, member variables and member functions
class HoffmannController {
public:
    HoffmannController(ros::NodeHandle* nodehandle); 
    void nl_steering(); // use state and desired state to compute twist command, and publish it
    double speed_cmd_fnc(double des_speed, double dist_err);
    double omega_cmd_fnc(double psi_strategy, double psi_state, double psi_path);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi); 
    double min_dang(double dang);  
    double sat(double x);
    double sign(double x);

private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor

    ros::Subscriber current_state_subscriber_; //topic to receive estimates of current robot state
    ros::Subscriber designed_speed_subscriber_;
    ros::Publisher cmd_publisher_; // sends twist commands to cmd_vel topic
    
    geometry_msgs::Twist twist_cmd_;
    
    //state variables, (x,y,psi) and (speed, omega)
    double state_x_;
    double state_y_;
    double state_psi_;
    double state_speed_;
    double state_omega_;
    
    geometry_msgs::Quaternion state_quat_;

    nav_msgs::Path des_path_;
    
    //state values from desired state; these will get filled in by desStateCallback 
    double des_state_x_;
    double des_state_y_;
    double des_state_psi_;   
    geometry_msgs::Quaternion des_state_quat_;

    double des_state_speed_;

    // private member methods:
    void initializeSubscribers(); 
    void initializePublishers();
 
    void gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose);
    void odomCallback(const nav_msgs::Odometry& odom_rcvd);
    void desSpeedCallback(const std_msgs::Float64& speed_rcvd);

    void generate_circle_path();
    void generate_track_path();
    double compute_des_state();
}; // end of class definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef
