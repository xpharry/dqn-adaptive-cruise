#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

using namespace std;

const double max_speed = 5.0;

double dist;
double cur_speed;

void distCallback(const std_msgs::Float32& distRcvd) {
    dist = distRcvd.data;
    ROS_INFO("Received dist = %f", dist);
}

void velCallback(const geometry_msgs::Twist& velRcvd) {
    cur_speed = velRcvd.linear.x;
    ROS_INFO("Received speed = %f", cur_speed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "adaptive_cruise");
    ros::NodeHandle n;
    ros::Subscriber current_dist_subscriber = n.subscribe("/catvehicle/distanceEstimator/dist", 1, distCallback);
    ros::Subscriber current_vel_subscriber = n.subscribe("/catvehicle/cmd_vel", 1, velCallback);
    ros::Publisher cmd_vel_pub = n.advertise<std_msgs::Float64>("/catvehicle/des_speed", 1000);

    ros::Rate loop_rate(20);

    double begin = ros::Time::now().toSec();
    while(ros::ok()) {
        double safe_dist = max(cur_speed * 3, 10.0);
        double cmd_speed = dist > safe_dist ? cur_speed + 0.05 * max_speed : cur_speed - 0.05 * max_speed;
        cmd_speed = cmd_speed > max_speed ? max_speed : cmd_speed;
        cmd_speed = cmd_speed < 0 ? 0 : cmd_speed;
        std_msgs::Float64 cmd_msg;
        cmd_msg.data = cmd_speed;
        cmd_vel_pub.publish(cmd_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}