#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cstdlib>
#include <string>
#include <sstream>
#include <chrono>
#include <signal.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed

void transformMsg(
    const mavros_msgs::PositionTarget& in,
    mavros_msgs::PositionTarget& out,
    const geometry_msgs::TransformStamped& transform)
    {
        out = in;
        tf2::doTransform(in.position, out.position, transform);
        out.header.stamp = transform.header.stamp;
        out.header.frame_id = transform.header.frame_id;
    }

class Node
{
    public:
        Node(): nh_priv_("~")
        {
            nh_priv_.getParam("altitude", takeoff_altitude);
            nh_priv_.getParam("takeoff_time", takeoff_time);
            nh_priv_.getParam("rotation_frequency", frequency);
            nh_priv_.getParam("radius", radius);
            nh_priv_.getParam("phase", phi);
            nh_priv_.getParam("id", ID);
            nh_priv_.getParam("center_x", xc);
            nh_priv_.getParam("center_y", yc);
            const char* env_id = std::getenv("UAV_ID");
            if(env_id != nullptr) {
                std::string id_value(env_id);
                std::istringstream iss(id_value);
                iss >> ID;
                std::cout << "Found env variable UAV_ID=" << ID << std::endl;
            }
	        std::cout << "ID: " << ID << std::endl;
            std::cout << "Trajectory: circle centered in (" << xc << ", " << yc << ") with r = " << radius << std::endl;
            omega = frequency * 2 * M_PI;
            state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 1, std::bind(&Node::state_cb, this, std::placeholders::_1));
            local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory_publisher/trajectory", 10);
            setpoint_pub = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
            vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
            odom_pub = nh_.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, std::bind(&Node::odom_cb, this, std::placeholders::_1));
            reference_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference", 10);
            timer_ = nh_.createTimer(ros::Duration(0.005), std::bind(&Node::timer_cb, this));
            traj_timer_ = nh_.createTimer(ros::Duration(0.5), std::bind(&Node::reference_trajectory, this));
            ros::Rate rate(20.0);
            while(ros::ok() && !current_state.connected && !odom_received){
                std::cout << "waiting for odometry ... \n";
                ros::spinOnce();
                rate.sleep();
            }
	    std::cout << "Wating 3 secs... \n";
	    for (int i = 0; i < 10; i++) {
		    ros::spinOnce();
		    rate.sleep();
	    }

            tf::StampedTransform transform;
            while(!offset_received) {
                try {
                    listener.lookupTransform("map", "uav"+std::to_string(ID)+"/local_origin", ros::Time(0), transform);
                    offset_x = transform.getOrigin().x();
                    offset_y = transform.getOrigin().y();
                    offset_z = transform.getOrigin().z();
		    std::cout << "offset: " << offset_x << ", " << offset_y <<", " << offset_z << std::endl;
                    offset_received = true;
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }

            while(!got_starting_pose) {
                try {
                    listener.lookupTransform("uav"+std::to_string(ID)+"/local_origin", "uav"+std::to_string(ID)+"/base_link", ros::Time(0), transform);
                    start_pose.pose.position.x = transform.getOrigin().x();
                    start_pose.pose.position.y = transform.getOrigin().y();
                    start_pose.pose.position.z = transform.getOrigin().z();
                    std::cout << "Starting pos: " << start_pose.pose.position.x << ", " << start_pose.pose.position.y << ", " << start_pose.pose.position.z << "\n";
                    got_starting_pose = true;
                }
                catch (tf::TransformException &ex) {
                    ROS_WARN("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    continue;
                }
            }

            // std::cout << "Odometry received!\n";
            // start_pose.pose.position.x = odom.pose.pose.position.x;
            // start_pose.pose.position.y = odom.pose.pose.position.y;
            // start_pose.pose.position.z = odom.pose.pose.position.z;

            takeoff_pose.pose.position.x = start_pose.pose.position.x;
            takeoff_pose.pose.position.y = start_pose.pose.position.y;
            takeoff_pose.pose.position.z = start_pose.pose.position.z + takeoff_altitude; 
	        std::cout << "Takeoff altitude: " << takeoff_pose.pose.position.z << std::endl;
            //send a few setpoints before starting
            for(int i = 100; ros::ok() && i > 0; --i){
                local_pos_pub.publish(start_pose);
                ros::spinOnce();
                rate.sleep();
            }

            last_request = ros::Time::now();
        
        }

        void stop();
        void timer_cb();
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
        void circle_trajectory(mavros_msgs::PositionTarget& msg, double t, double r, double w);
        void global_circle_trajectory(mavros_msgs::PositionTarget& msg, double t, double r, double w);
        void reference_trajectory();

    private:
        // ROS
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        ros::Timer timer_;
        ros::Timer traj_timer_;
        ros::Subscriber state_sub;
        ros::Subscriber odom_pub;
        ros::Publisher local_pos_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient landing_client;
        ros::ServiceClient set_mode_client;
        ros::Publisher vel_pub;
        ros::Publisher setpoint_pub;
        ros::Publisher traj_pub_;
        ros::Publisher reference_pub_;
        ros::Time last_request;
        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseStamped takeoff_pose;
        geometry_msgs::PoseStamped reference_pose;
        nav_msgs::Odometry odom;
        tf::TransformListener listener;
    
        bool odom_received = false;
        bool offset_received = false;
        bool got_starting_pose = false;
        bool takeoff_complete = false;
        int ID = 0;
        double takeoff_time = 10.0;
        double takeoff_altitude = 3.0;
        double t_start = 0.0;
        double frequency = 0.1;
        double omega;
        double phi = 0.0;
        double radius = 3.0;
        double xc = 0.0;                    // center coordinates (in map frame)
        double yc = 0.0;
        double offset_x = 0.0;              // difference between common map origin and local origin
        double offset_y = 0.0;
	    double offset_z = 0.0;
};

void Node::stop()
{
    // Land the drone
    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.min_pitch = 0.0;
    land_cmd.request.yaw = 0.0;
    land_cmd.request.latitude = 0.0;
    land_cmd.request.longitude = 0.0;
    land_cmd.request.altitude = 0.0;
    std::cout << "Shutdown required! Landing the drone...\n";
    if (landing_client.call(land_cmd) && land_cmd.response.success){
        ROS_INFO("Landing command received");
    }
    ros::Duration(0.1).sleep();
    ros::shutdown();
}

void Node::state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void Node::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
	odom = *msg;
	if (odom.pose.pose.position.x != 0.0 && odom.pose.pose.position.y != 0.0 && odom.pose.pose.position.z != 0.0){
		odom_received = true;
	}
}

void Node::circle_trajectory(mavros_msgs::PositionTarget& msg, double t, double r=3.0, double w=1.0) {
    double wt = w*t;
    msg.position.x = takeoff_pose.pose.position.x - offset_x + r * cos(wt + phi);
    msg.position.y = takeoff_pose.pose.position.y - offset_y + r * sin(wt + phi);
    msg.position.z = takeoff_pose.pose.position.z - offset_z;
    msg.velocity.x = -r*w * sin(wt + phi);
    msg.velocity.y = r*w * cos(wt + phi);
    msg.velocity.z = 0.8 * (takeoff_pose.pose.position.z - offset_z - odom.pose.pose.position.z);
    msg.acceleration_or_force.x = -r*pow(w, 2) * cos(wt + phi);
    msg.acceleration_or_force.y = -r*pow(w, 2) * sin(wt + phi);
    msg.acceleration_or_force.z = msg.velocity.z - 0.8 * odom.twist.twist.linear.z;
    msg.yaw = atan2(msg.velocity.y, msg.velocity.x);
}

void Node::global_circle_trajectory(mavros_msgs::PositionTarget& msg, double t, double r=3.0, double w=1.0) {
    double wt = w*t;
    msg.position.x = xc + r * cos(wt + phi);
    msg.position.y = yc + r * sin(wt + phi);
    //msg.position.z = takeoff_altitude;
    msg.position.z = takeoff_pose.pose.position.z;
    msg.velocity.x = -r*w * sin(wt + phi);
    msg.velocity.y = r*w * cos(wt + phi);
    msg.velocity.z = 0.8 * (takeoff_pose.pose.position.z - odom.pose.pose.position.z);
    msg.acceleration_or_force.x = -r*pow(w, 2) * cos(wt + phi);
    msg.acceleration_or_force.y = -r*pow(w, 2) * sin(wt + phi);
    msg.acceleration_or_force.z = msg.velocity.z - 0.8 * odom.twist.twist.linear.z;
    msg.yaw = atan2(msg.velocity.y, msg.velocity.x);
}

void Node::reference_trajectory() {
    if (odom_received) {
        nav_msgs::Path traj_msg;
        auto time_now = ros::Time::now();
        int num_steps = 200;
        //traj_msg.header.frame_id = "uav"+std::to_string(ID)+"/odom";
        traj_msg.header.frame_id = "map";
	    traj_msg.header.stamp = time_now;
        for (double i = 0.0; i < 2*M_PI; i+=2*M_PI/num_steps) {
            double wt = omega * i;
            geometry_msgs::PoseStamped p;
            p.header.stamp = time_now;
            p.header.frame_id = "map";
            p.pose.position.x = xc + radius * cos(i);
            p.pose.position.y = yc + radius * sin(i);
            p.pose.position.z = takeoff_pose.pose.position.z;

            traj_msg.poses.push_back(p);
        }
        traj_pub_.publish(traj_msg);
    }
}


void Node::timer_cb()
{
    if(!odom_received && !got_starting_pose){
        return;
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // std::cout << "Current state: " << current_state << std::endl;
    if (current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(2.0))
    {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
        {
            ROS_INFO("Offboard enabled.");
        }
        last_request = ros::Time::now();
    } else
    {
        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed.");
            }
            last_request = ros::Time::now();
        }
    }

    // Wait 10 seconds after takeoff request
    if (ros::Time::now() - last_request < ros::Duration(takeoff_time))
    {
        //ROS_INFO("Taking off.");
        takeoff_pose.header.frame_id = "map";
        takeoff_pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(takeoff_pose);
        reference_pose.header.frame_id = "uav"+std::to_string(ID)+"/local_origin";
        reference_pose.header.stamp = ros::Time::now();
        reference_pose.pose = takeoff_pose.pose;
        reference_pub_.publish(reference_pose);
        t_start = ros::Time::now().toSec();
    } else {
        double t_now = ros::Time::now().toSec();
        mavros_msgs::PositionTarget global_msg;
        global_msg.header.frame_id = "map";
        global_msg.header.stamp = ros::Time::now();
        global_msg.coordinate_frame = 1;
        global_circle_trajectory(global_msg, t_now-t_start+takeoff_time, radius, omega);
        
        // Convert from map to local origin
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        try {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
                "uav"+std::to_string(ID)+"/local_origin",
                "map",
                ros::Time(0),
                ros::Duration(0.1)
            );

            mavros_msgs::PositionTarget local_msg;
            transformMsg(global_msg, local_msg, transformStamped);
            setpoint_pub.publish(local_msg);
            reference_pose.header.frame_id = "uav"+std::to_string(ID)+"/local_origin";
            reference_pose.header.stamp = ros::Time::now();
            reference_pose.pose.position.x = local_msg.position.x;
            reference_pose.pose.position.y = local_msg.position.y;
            reference_pose.pose.position.z = local_msg.position.z;
            reference_pose.pose.orientation = tf2::toMsg(tf2::Quaternion(
                0.0, 0.0, local_msg.yaw
            ));
            reference_pub_.publish(reference_pose);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Transform lookup failed: %s", ex.what());
        }

        
    }
    // std::cout << "z: " << odom.pose.pose.position.z << std::endl;
    // std::cout << "target z: " << takeoff_pose.pose.position.z << std::endl;
} 


void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lemniscate_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    //Controller node_controller;
    auto node_controller = std::make_shared<Node>();

    while (!node_shutdown_request){
        ros::spinOnce();
    }
    node_controller->stop();

    //ros::spin();
    //do pre-shutdown tasks
    if (ros::ok())
    {
        ROS_WARN("ROS HAS NOT BEEN PROPERLY SHUTDOWN, it is being shutdown again.");
        ros::shutdown();
    }

    

    

    return 0;
}
