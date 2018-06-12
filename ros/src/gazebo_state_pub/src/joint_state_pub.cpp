#include <ros/ros.h> 
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <stdio.h>  
#include <math.h>
#include <random>

geometry_msgs::Quaternion quat;
ros::Publisher joint_state_pub;
std::string target_model;

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_y = quaternion.y;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_y, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void link_state_CB(const gazebo_msgs::LinkStates& link_states) {
    sensor_msgs::JointState joint_state;

    std::string link_cart = "robot::cart";
    std::string link_pole = "robot::pole";

    int n_models = link_states.name.size();
    int imodel, icart, ipole;
    //ROS_INFO("there are %d models in the transmission",n_models);

    bool found_cart = false, found_pole = false;
    for(imodel = 0; imodel < n_models; imodel++) {
        std::string model_name(link_states.name[imodel]); 
        if (model_name.compare(link_cart) == 0) {
            //ROS_INFO("found match: mobot is model %d",imodel);
            found_cart = true;
            icart = imodel;
        }
        if (model_name.compare(link_pole) == 0) {
            //ROS_INFO("found match: mobot is model %d",imodel);
            found_pole = true;
            ipole = imodel;
        }
    }

    if(found_cart && found_pole) {
        joint_state.name.push_back(link_cart);
        joint_state.name.push_back(link_pole);

        double position_cart, position_pole;
        position_cart = link_states.pose[icart].position.x;
        position_pole = convertPlanarQuat2Phi(link_states.pose[ipole].orientation);
        joint_state.position.push_back(position_cart);
        joint_state.position.push_back(position_pole);

        double vel_cart, vel_pole;
        vel_cart = link_states.twist[icart].linear.x;
        vel_pole = link_states.twist[ipole].angular.y;
        joint_state.velocity.push_back(vel_cart);
        joint_state.velocity.push_back(vel_pole);

        joint_state.effort.push_back(0.0);
        joint_state.effort.push_back(0.0);

        joint_state_pub.publish(joint_state); //publish noisy values
    } else {
        ROS_WARN("state of model not found");
    }
} 

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_model_state_publisher");
    ros::NodeHandle nh;

    ros::param::get("~target_model", target_model);

    joint_state_pub = nh.advertise<sensor_msgs::JointState>(target_model+"/joint_states", 1);

    ros::Subscriber state_sub = nh.subscribe("/gazebo/link_states", 1, link_state_CB);

    ros::spin();
}
