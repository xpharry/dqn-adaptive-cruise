#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

using namespace std;

double MAX_SPEED = 22.35;
double MIN_SPEED = 8;

// saturation function, values -1 to 1
double sat(double x) {
    if (x > MAX_SPEED) {
        return MAX_SPEED;
    }
    if (x < MIN_SPEED) {
        return MIN_SPEED;
    }
    return x;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "des_velocity");
    ros::NodeHandle n;
    ros::Publisher des_speed_pub = n.advertise<std_msgs::Float64>("/des_speed", 1000);
    // ros::Publisher des_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/catvehicle/des_vel", 1000);
    ros::Rate loop_rate(5);

    // speed = 2 + 2 * sin(w*t)
    double prev_speed = 10;
    double curr_speed = 0;

    ofstream myfile;
    myfile.open ("/home/peng/rand_speed_history.txt");
    
    while(ros::ok()) {
        // speed value
        double addon = rand() % 100;
        addon = addon / 100 * 4 - 2;
        curr_speed = sat(prev_speed + addon);
        ROS_INFO("adding = %f", addon);

        // desired speed
        std_msgs::Float64 msg;
        msg.data = curr_speed;
        ROS_INFO("des_speed = %f", msg.data);
        des_speed_pub.publish(msg);

        myfile << curr_speed << "\n";
        prev_speed = curr_speed;

        ros::spinOnce();
        loop_rate.sleep();
    }
    myfile.close();

    return 0;
}