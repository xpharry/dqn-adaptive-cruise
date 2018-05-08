#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "des_velocity");
    ros::NodeHandle n;
    ros::Publisher des_speed_pub = n.advertise<std_msgs::Float64>("/des_speed", 1000);
    // ros::Publisher des_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/catvehicle/des_vel", 1000);
    ros::Rate loop_rate(20);

    // speed = 2 + 2 * sin(w*t)
    double speed_mag = 5;
    double bias = 10;
    double omega = 2 * M_PI / 3.6;

    double begin = ros::Time::now().toSec();
    while(ros::ok()) {
        // speed value
        double time = ros::Time::now().toSec();
        ROS_INFO("elapsed time = %f", time - begin);
        float speed = bias + speed_mag * sin(omega*(time - begin));

        // desired speed
        std_msgs::Float64 msg;
        msg.data = speed;
        ROS_INFO("des_speed = %f", msg.data);
        des_speed_pub.publish(msg);

        // desired velocity
//        geometry_msgs::TwistStamped vel;
//        vel.header.stamp = ros::Time::now();
//        vel.header.frame_id = "/catvehicle/odom";
//        vel.twist.linear.x = speed;
//        des_velocity_pub.publish(vel);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}