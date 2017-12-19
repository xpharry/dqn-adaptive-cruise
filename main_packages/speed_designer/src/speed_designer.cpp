#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher speed_pub = n.advertise<std_msgs::Float64>("/catvehicle/des_speed", 1000);
    ros::Rate loop_rate(20);

    // speed = 2 + 2 * sin(w*t)
    double speed_mag = 2;
    double bias = 3;
    double omega = 2 * M_PI / 3.6;

    double begin = ros::Time::now().toSec();
    while(ros::ok()) {
        std_msgs::Float64 msg;
        double time = ros::Time::now().toSec();
        ROS_INFO("elapsed time = %f", time - begin);
        msg.data = bias + speed_mag * sin(omega*(time - begin));
        ROS_INFO("des_speed = %f", msg.data);
        speed_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}