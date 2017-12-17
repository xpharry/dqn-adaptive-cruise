#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string vehicle_name;

double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void odomCallback(const nav_msgs::Odometry& odom_rcvd){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", vehicle_name+"/odom"));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2){ROS_ERROR("need vehicle name as argument"); return -1;};
    vehicle_name = argv[1];

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(vehicle_name+"/odom", 10, &odomCallback);

    ros::spin();
    return 0;
};