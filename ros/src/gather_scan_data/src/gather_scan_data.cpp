// Author: Peng Xu
// This (very simple) node reads a laser scan, and 
// store the distance in a file

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>

std::vector<float> ranges;
double dist;
double angle;
double speed;
double angle_min;
double angle_max;

// This very simple callback looks through the data array, and then
// returns the value (not the index) of that distance
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ranges = scan->ranges;
    dist = scan->range_max;
    angle = scan->angle_min;
    float angle_tmp = scan->angle_min;
    float inc = scan->angle_increment;
    for(std::vector<float>::const_iterator it = scan->ranges.begin();
        it != scan->ranges.end(); it++, angle_tmp = angle_tmp+inc) {
        if(dist > *it && *it > scan->range_min 
            && angle_tmp > angle_min && angle_tmp < angle_max) {
            dist = *it;
            angle = angle_tmp;
        }
    }
}

void velCallback(const geometry_msgs::Twist& velRcvd) {
    speed = velRcvd.linear.x;
    // ROS_INFO("Received speed = %f", cur_speed);
}

int main( int argc, char **argv ) {
	ros::init(argc, argv, "ScanData");
	ros::NodeHandle n("~");

    std::string file_path, file_name;
    file_name = "scan_data.csv";
    bool get_file_path;
    get_file_path = n.getParam("file_path", file_path);

    char resolved_path[PATH_MAX];
    realpath(file_path.c_str(), resolved_path);
    ROS_INFO("File path = %s", resolved_path);

    file_path = resolved_path;
    // file_path = file_path.substr(0, file_path.length()-2);
    ROS_INFO("File path = %s", file_path.c_str());

    if(get_file_path) {
        file_name = file_path + "/" + file_name;
    }
    ROS_INFO("File name  = %s", file_name.c_str());

  	// we also want to subscribe to the signaller
  	ros::Subscriber scan_sub = n.subscribe("/catvehicle/front_laser_points", 1, &scanCallback);
    ros::Subscriber vel_sub = n.subscribe("/catvehicle/cmd_vel", 1, &velCallback);

  	// run at 50Hz?
  	ros::Rate loop_rate(50);

	while(ros::ok()) {
        std::ofstream scan_file;
        scan_file.open(file_name.c_str(), std::ofstream::out);
        if(scan_file.is_open() && !ranges.empty()) {
            ROS_INFO("Writing data to file ...");
            for(int i = 0; i < ranges.size(); i++) {
                scan_file << ranges[i] << ", ";
            }
            double safe_dist = std::max(speed * 3, 10.0);
            bool isbrake = dist < safe_dist ? true : false;
            scan_file << speed << ", ";
            scan_file << isbrake << "\n";
            scan_file.close();
        }
		ros::spinOnce();
		loop_rate.sleep();
	}

	return EXIT_SUCCESS;
}

