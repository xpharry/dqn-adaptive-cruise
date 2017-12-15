//hoffmann_controller.cpp:
//Peng Xu, Dec 2017
//subscribes to topics for desired state and actual state
// invokes a nonlinear steering algorithm to command speed and spin on cmd_vel topic

// this header incorporates all the necessary #include files and defines the class "HoffmannController"
#include "hoffmann_controller.h"

//CONSTRUCTOR:  
HoffmannController::HoffmannController(ros::NodeHandle* nodehandle):nh_(*nodehandle) { // constructor
    ROS_INFO("in class constructor of HoffmannController");
    initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
    initializePublishers();
    generate_path();
    
    state_psi_ = 1000.0; // put in impossible value for heading; 
    //test this value to make sure we have received a viable state message
    ROS_INFO("waiting for valid state message...");
    while (state_psi_ > 500.0) {
        ros::Duration(0.5).sleep(); // sleep for half a second
        std::cout << ".";
        ros::spinOnce();
    }
    ROS_INFO("constructor: got a state message");    
      
    //initialize desired state;  can be changed dynamically by publishing to topic /desState
    des_state_speed_ = MAX_SPEED; //can make dynamic via des_state_rcvd.twist.twist.linear.x;
    
    // hard code a simple path: the world x axis
    des_state_x_ = 0.0; 
    des_state_y_ = 0.0; 
    des_state_psi_ = 0.0; 

    //initialize the twist command components, all to zero
    twist_cmd_.linear.x = 0.0;
    twist_cmd_.linear.y = 0.0;
    twist_cmd_.linear.z = 0.0;
    twist_cmd_.angular.x = 0.0;
    twist_cmd_.angular.y = 0.0;
    twist_cmd_.angular.z = 0.0;
}

//member helper function to set up subscribers;
void HoffmannController::initializeSubscribers() {
    ROS_INFO("Initializing Subscribers: gazebo state and desState");
    
    //subscribe to gazebo messages; ONLY works in simulation
    current_state_subscriber_ = nh_.subscribe("/gazebo_mobot_pose", 1, &HoffmannController::gazeboPoseCallback, this);
    // alternately we can use the topic "/catvehicle/odom"
    // current_state_subscriber_ = nh_.subscribe("/catvehicle/odom", 1, &HoffmannController::odomCallback, this); 
}


//member helper function to set up publishers;
void HoffmannController::initializePublishers() {
    ROS_INFO("Initializing Publishers");
    cmd_publisher_ = nh_.advertise<geometry_msgs::Twist>("/catvehicle/cmd_vel", 1, true); // commands the robot!
}

//receive publications from gazebo via node mobot_gazebo_state;
// this stands in for a state estimator; for a real system need to create such a node
void HoffmannController::gazeboPoseCallback(const geometry_msgs::Pose& gazebo_pose) {
   state_x_ = gazebo_pose.position.x; //copy the state to member variables of this object
   state_y_ = gazebo_pose.position.y;
   state_quat_ = gazebo_pose.orientation;
   state_psi_ = convertPlanarQuat2Phi(state_quat_);
}

void HoffmannController::odomCallback(const nav_msgs::Odometry& odom_rcvd) {
    state_x_ = odom_rcvd.pose.pose.position.x;
    state_y_ = odom_rcvd.pose.pose.position.y;
    state_quat_ = odom_rcvd.pose.pose.orientation;
    state_psi_ = convertPlanarQuat2Phi(state_quat_); // cheap conversion from quaternion to heading for planar motion
}

//utility fnc to compute min delta angle, accounting for periodicity
double HoffmannController::min_dang(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}

// saturation function, values -1 to 1
double HoffmannController::sat(double x) {
    if (x > 1.0) {
        return 1.0;
    }
    if (x < -1.0) {
        return -1.0;
    }
    return x;
}

double HoffmannController::sign(double x) {
    if (x > 1.0) {
        return 1.0;
    }
    if (x < -1.0) {
        return -1.0;
    }
    return x;
}

//some conversion utilities:
double HoffmannController::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

geometry_msgs::Quaternion HoffmannController::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}


// HERE IS THE STEERING ALGORITHM: USE DESIRED AND ACTUAL STATE TO COMPUTE AND PUBLISH CMD_VEL
void HoffmannController::nl_steering() {
    double controller_speed;
    double controller_omega;
    
    compute_des_state();

    double dx = des_state_x_ - state_x_;  //x-error relative to desired path
    double dy = des_state_y_ - state_y_;  //y-error
    double tx = cos(des_state_psi_); // [tx,ty] is tangent of desired path
    double ty = sin(des_state_psi_); 
    double nx = ty; //components [nx, ny] of normal to path, points to left of desired heading
    double ny = -tx;

    double signnn = sign(dx*nx + dy*ny);
    double offset = sqrt(dx*dx + dy*dy);
    double lateral_err_ = signnn * offset;
    double trip_dist_err = dx*tx + dy*ty; // progress error: if positive, then we are ahead of schedule    
    //heading error: if positive, should rotate -omega to align with desired heading
    double heading_err = min_dang(state_psi_ - des_state_psi_);    
    
    controller_speed = speed_cmd_fnc(des_state_speed_, lateral_err_); //default...should speed up/slow down appropriately
    controller_omega = omega_cmd_fnc(heading_err, lateral_err_, controller_speed);

    // send out our speed/spin commands:
    twist_cmd_.linear.x = controller_speed;
    twist_cmd_.angular.z = controller_omega;
    cmd_publisher_.publish(twist_cmd_);  
        
    // DEBUG OUTPUT...
    ROS_INFO("lateral err = %f,  \theading err = %f", lateral_err_, heading_err);
    ROS_INFO("state_x_ = %f,     \tstate_y_ = %f,     \tstate_psi_ = %f", state_x_, state_y_, state_psi_);
    ROS_INFO("des_state_x_ = %f, \tdes_state_y_ = %f, \tdes_state_psi_ = %f", des_state_x_, des_state_y_, des_state_psi_);
    //END OF DEBUG OUTPUT   
}

double HoffmannController::speed_cmd_fnc(double des_speed, double dist_err) {
    return dist_err > 4 ? 0 : des_speed;
    // return des_speed;
}

double HoffmannController::omega_cmd_fnc(double heading_err, double offset_err, double des_speed) {
    double omega_cmd;
    omega_cmd = heading_err + atan2(K_PSI * offset_err, des_speed);
    // omega_cmd = MAX_OMEGA * sat(omega_cmd / MAX_OMEGA); // saturate omega command at specified limits
    return -omega_cmd;
}

void HoffmannController::generate_path() {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "world";

    const float ra = 36.6;
    const int nedges = 301;
    double phi0 = -M_PI;     
    for(int i = 0; i < nedges; i++) {

        // point i
        geometry_msgs::Pose pose;
        geometry_msgs::Quaternion quat;

        double phi = min_dang(phi0 + 2.0 * M_PI / nedges * i);
        pose.position.x = ra * cos(phi) - ra; // say desired x-coord is 5
        pose.position.y = ra * sin(phi);
        pose.position.z = 0.0; // let's hope so!
        
        double theta = min_dang(phi + M_PI/2.0);
        quat = convertPlanarPhi2Quaternion(theta);
        pose.orientation = quat;

        pose_stamped.pose = pose;
        des_path_.poses.push_back(pose_stamped);            
    }
}

double HoffmannController::compute_des_state() {
    // codegen
    double e = DBL_MAX;
    int index = 0;

    // find the nearest x,y
    for(int i = 0; i < des_path_.poses.size(); i++) {
        double dx = des_path_.poses[i].pose.position.x - state_x_;
        double dy = des_path_.poses[i].pose.position.y - state_y_;
        double etmp = sqrt(dx*dx + dy*dy);
        if(etmp < e) {
            e = etmp;
            index = i;
        }
    }

    des_state_x_ = des_path_.poses[index].pose.position.x;
    des_state_y_ = des_path_.poses[index].pose.position.y;
    des_state_quat_ = des_path_.poses[index].pose.orientation;
    des_state_psi_ = min_dang(convertPlanarQuat2Phi(des_state_quat_));
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "HoffmannController"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type HoffmannController");
    //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    HoffmannController HoffmannController(&nh);  
 
    ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate, e.g. 50Hz
   
    ROS_INFO:("starting steering algorithm");
    while (ros::ok()) {
        // compute and publish twist commands 
        HoffmannController.nl_steering(); 
        ros::spinOnce();
        sleep_timer.sleep();
    }
    return 0;
} 

