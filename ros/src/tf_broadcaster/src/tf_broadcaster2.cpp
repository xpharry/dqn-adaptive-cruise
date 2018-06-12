#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/LinkState.h>


double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_y = quaternion.y;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_y, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

void stateCallback(const gazebo_msgs::LinkStates& state_rcvd){
    std::string link_cart = "robot::cart";
    std::string link_pole = "robot::pole";
    gazebo_msgs::LinkState state_cart;
    gazebo_msgs::LinkState state_pole;
    tf::Vector3 origin;
    tf::Vector3 rpy;
    tf::Quaternion quat;

    int n_models = state_rcvd.name.size();
    int imodel, icart, ipole;
    //ROS_INFO("there are %d models in the transmission", n_models);

    bool found_cart = false, found_pole = false;
    for(imodel = 0; imodel < n_models; imodel++) {
        std::string model_name(state_rcvd.name[imodel]); 
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
        state_cart.link_name = link_cart;
        state_cart.pose = state_rcvd.pose[icart];
        state_cart.twist = state_rcvd.twist[icart];
        state_cart.reference_frame = "world";

        state_pole.link_name = link_pole;
        state_pole.pose = state_rcvd.pose[ipole];
        state_pole.twist = state_rcvd.twist[ipole];
        state_pole.reference_frame = link_cart;

        origin[0] = state_cart.pose.position.x;
        origin[1] = state_cart.pose.position.y;
        origin[2] = state_cart.pose.position.z;

        quat[0] = state_pole.pose.orientation.x;
        quat[1] = state_pole.pose.orientation.y;
        quat[2] = state_pole.pose.orientation.z;
        quat[3] = state_pole.pose.orientation.w;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(origin);
        tf::Quaternion q = quat;
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "pole"));
    } else {
        ROS_WARN("state of model not found");
    }    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_broadcaster2");

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/gazebo/link_states", 10, &stateCallback);

    ros::spin();

    return 0;
};