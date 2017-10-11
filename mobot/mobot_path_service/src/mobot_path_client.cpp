//path_client:
// illustrates how to send a request to the path_service service

#include <ros/ros.h>
#include <mobot_path_service/MyPathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

#define PI 3.1415926

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mobot_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<mobot_path_service::MyPathSrv>("mobot_path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    mobot_path_service::MyPathSrv path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;

    // initialization
    pose.position.x = 0.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 0.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;

    // subgoal 1
    quat = convertPlanarPhi2Quaternion(0); // get a quaterion corresponding to this heading
    pose_stamped.pose.orientation = quat;   
    pose_stamped.pose.position.y = 10.0; // say desired y-coord is 1.0
    path_srv.request.nav_path.poses.push_back(pose_stamped);
 
    // subgoal 2
    quat = convertPlanarPhi2Quaternion(PI/2); // get a quaterion corresponding to this heading
    pose_stamped.pose.orientation = quat;   
    pose_stamped.pose.position.y = 0.0; // say desired y-coord is 1.0
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // subgoal 3
    quat = convertPlanarPhi2Quaternion(0); // get a quaterion corresponding to this heading
    pose_stamped.pose.orientation = quat;   
    pose_stamped.pose.position.y = 10.0; // say desired y-coord is 1.0
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // // subgoal 4
    // quat = convertPlanarPhi2Quaternion(PI); // get a quaterion corresponding to this heading
    // pose_stamped.pose.orientation = quat;   
    // pose_stamped.pose.position.y = 3.2; // say desired y-coord is 1.0
    // path_srv.request.nav_path.poses.push_back(pose_stamped);

    // // subgoal 5
    // quat = convertPlanarPhi2Quaternion(PI/2); // get a quaterion corresponding to this heading
    // pose_stamped.pose.orientation = quat;   
    // pose_stamped.pose.position.y = 3.2; // say desired y-coord is 1.0
    // path_srv.request.nav_path.poses.push_back(pose_stamped);

    // // subgoal 6
    // quat = convertPlanarPhi2Quaternion(PI); // get a quaterion corresponding to this heading
    // pose_stamped.pose.orientation = quat;   
    // pose_stamped.pose.position.y = 3; // say desired y-coord is 1.0
    // path_srv.request.nav_path.poses.push_back(pose_stamped);

    // quat = convertPlanarPhi2Quaternion(PI);
    // pose_stamped.pose.orientation = quat;  
    // //desired position is not updated...just the desired heading  
    // path_srv.request.nav_path.poses.push_back(pose_stamped);

    client.call(path_srv);

    return 0;
}
