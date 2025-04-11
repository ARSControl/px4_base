/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>

mavros_msgs::State current_state;
nav_msgs::Odometry odom;
bool odom_received = false;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
	odom = *msg;
	if (odom.pose.pose.position.z != 0.0){
		odom_received = true;
		std::cout << "odom: " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z << std::endl;
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>
	    ("mavros/setpoint_velocity/cmd_vel", 10);

    ros::Subscriber odom_pub = nh.subscribe<nav_msgs::Odometry>
	    ("mavros/local_position/odom", 10, odom_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected && !odom_received){
        ros::spinOnce();
        rate.sleep();
	std::cout << "Spinning...\n";
    }
    // while(ros::ok() && !odom_received){
    //	std::cout << "Waiting 5 seconds...\n";
    //	ros::Duration(5.0).sleep();
    //}
    geometry_msgs::PoseStamped start_pose;
    start_pose.pose.position.x = odom.pose.pose.position.x;
    start_pose.pose.position.y = odom.pose.pose.position.y;
    start_pose.pose.position.z = odom.pose.pose.position.z;
    std::cout << "Starting pos: " << start_pose.pose.position.x << ", " << start_pose.pose.position.y << ", " << start_pose.pose.position.z << "\n";

    geometry_msgs::PoseStamped takeoff_pose;
    takeoff_pose.pose.position.x = start_pose.pose.position.x;
    takeoff_pose.pose.position.y = start_pose.pose.position.y;
    takeoff_pose.pose.position.z = start_pose.pose.position.z + 2.0; 
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(start_pose);
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped vel;
    vel.twist.linear.x = 0.0;
    vel.twist.linear.y = 0.0;
    vel.twist.linear.z = 0.0;
    vel_pub.publish(vel);


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
	    
            	//vel_pub.publish(vel);
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
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

	//std::cout << "Taking off ... \n";
	local_pos_pub.publish(takeoff_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
