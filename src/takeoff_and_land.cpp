#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

#include <chrono>
#include <signal.h>

#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed

class Node
{
    public:
        Node(): nh_priv_("~")
        {
            state_sub = nh_.subscribe<mavros_msgs::State>("mavros/state", 1, std::bind(&Node::state_cb, this, std::placeholders::_1));
            local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);
            odom_pub = nh_.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, std::bind(&Node::odom_cb, this, std::placeholders::_1));
            timer_ = nh_.createTimer(ros::Duration(0.01), std::bind(&Node::timer_cb, this));
            ros::Rate rate(20.0);
            while(ros::ok() && !current_state.connected && !odom_received){
                std::cout << "waiting for odometry ... \n";
                ros::spinOnce();
                rate.sleep();
            }
            std::cout << "Odometry received!\n";
            start_pose.pose.position.x = odom.pose.pose.position.x;
            start_pose.pose.position.y = odom.pose.pose.position.y;
            start_pose.pose.position.z = odom.pose.pose.position.z;
            std::cout << "Starting pos: " << start_pose.pose.position.x << ", " << start_pose.pose.position.y << ", " << start_pose.pose.position.z << "\n";

            takeoff_pose.pose.position.x = start_pose.pose.position.x;
            takeoff_pose.pose.position.y = start_pose.pose.position.y;
            takeoff_pose.pose.position.z = start_pose.pose.position.z + 2.0; 

            //send a few setpoints before starting
            for(int i = 100; ros::ok() && i > 0; --i){
                local_pos_pub.publish(start_pose);
                ros::spinOnce();
                rate.sleep();
            }
        
        }

        void stop();
        void timer_cb();
        void state_cb(const mavros_msgs::State::ConstPtr& msg);
        void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);

    private:
        // ROS
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        ros::Timer timer_;
        ros::Subscriber state_sub;
        ros::Subscriber odom_pub;
        ros::Publisher local_pos_pub;
        ros::ServiceClient arming_client;
        ros::ServiceClient landing_client;
        ros::ServiceClient set_mode_client;
        ros::Publisher vel_pub;
        ros::Time last_request;

        mavros_msgs::State current_state;
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::PoseStamped takeoff_pose;
        nav_msgs::Odometry odom;
        bool odom_received = false;

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
	if (odom.pose.pose.position.z != 0.0){
		odom_received = true;
	}
}

void Node::timer_cb()
{
    if(!odom_received){
        return;
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( current_state.mode != "OFFBOARD" &&
        (ros::Time::now() - last_request > ros::Duration(5.0))){
    if( set_mode_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    } else {
        if( !current_state.armed &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                ROS_INFO("Vehicle armed. Waiting 3 seconds for takeoff.");
            }
            last_request = ros::Time::now();
        }
    }

    if(ros::Time::now() - last_request > ros::Duration(3.0)) {   
        local_pos_pub.publish(takeoff_pose);
    }
}

void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "takeoff_node", ros::init_options::NoSigintHandler);
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
