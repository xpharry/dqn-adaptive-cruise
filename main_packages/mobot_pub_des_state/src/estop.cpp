#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

bool hardware_estop = false; // Indicates the hardware e-stop is active
bool lidar_alarm = false;  // Indicates the lidar e-stop is active
bool estop = false;
ros::ServiceClient estop_client;
ros::ServiceClient estop_clear_client;
ros::Subscriber motor_subscriber;
ros::Subscriber lidar_subscriber;

void lidarCallback(std_msgs::Bool alarm) {
    //lidar alam activated, set bool to trigger stop
    if(alarm.data) {
        ROS_INFO("LIDAR e-stop detected");
        lidar_alarm = true;
        std_srvs::Trigger trigger;
        if (!estop) {
            estop_client.call(trigger);
        }
        estop = true;
    }
    // If no LIDAR e-stop is called, only reset the estop_server if there is also no hardware e-stop
    else if (!hardware_estop && estop)
    {
        ROS_INFO("All clear, resetting e-stop");
        lidar_alarm = false;
        estop = false;
        std_srvs::Trigger trigger;
        estop_clear_client.call(trigger);
    }
    // If there is a hardware e-stop, don't reset service, but indicate there is no longer a LIDAR alarm
    else if (estop)
    {
        ROS_INFO("LIDAR e-stop deactivated but hardware e-stop still present");
        lidar_alarm = false;
    }
}

void motorCallback(std_msgs::Bool msg) {
    // If a hardware e-stop is called, trigger the estop_server
    if (!msg.data)
    {
        ROS_INFO("Hardware e-stop detected");
        hardware_estop = true;
        std_srvs::Trigger trigger;
        if (!estop) {
            estop_client.call(trigger);
        }
        estop = true;
    }
    // If no hardware e-stop is called, only reset the estop_server if there is also no lidar_alarm
    else if (!lidar_alarm & estop)
    {
        ROS_INFO("All clear, resetting e-stop");
        hardware_estop = false;
        estop = false;
        std_srvs::Trigger trigger;
        estop_clear_client.call(trigger);
    }
    // If there is a lidar_alarm, don't reset service, but indicate there is no longer a hardware e-stop
    else if (estop)
    {
        ROS_INFO("Hardware e-stop deactivated but lidar_alarm still present");
        hardware_estop = false;
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "estop");
	ros::NodeHandle nh_;

	motor_subscriber = nh_.subscribe("motors_enabled", 1, motorCallback);
    lidar_subscriber = nh_.subscribe("lidar_alarm", 1, lidarCallback); //need to add alarm from somebody's code
	estop_client = nh_.serviceClient<std_srvs::Trigger>("estop_service");
    estop_clear_client = nh_.serviceClient<std_srvs::Trigger>("clear_estop_service");

    ros::spin();
}