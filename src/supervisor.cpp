#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/TransformStamped.h>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <math.h>
#include <chrono>
#include <signal.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>


#define M_PI   3.14159265358979323846  /*pi*/

using namespace std::chrono_literals;
using std::placeholders::_1;

sig_atomic_t volatile node_shutdown_request = 0;    //signal manually generated when ctrl+c is pressed


// Global vars
double EARTH_RADIUS = 6378137.0;

Eigen::Vector3d global_to_local(double lat, double lon, double alt,
                                double origin_lat, double origin_lon, double origin_alt)
{
    double d_lat = (lat - origin_lat) * M_PI / 180.0;
    double d_lon = (lon - origin_lon) * M_PI / 180.0;
    
    double lat1 = origin_lat * M_PI / 180.0;
    double lat2 = lat * M_PI / 180.0;

    Eigen::Vector3d local_pos;
    local_pos(0) = EARTH_RADIUS * d_lon * cos((lat1 + lat2) / 2);
    local_pos(1) = EARTH_RADIUS * d_lat;
    local_pos(2) = alt - origin_alt;

    return local_pos;
    
}



class Supervisor
{
    public:
        Supervisor() : nh_priv_("~")
        {
            // ---------------- ROS params -----------------
            this->nh_.getParam("robots_ids", IDS);
            std::cout << "Got IDS: ";
            for (auto id : IDS) {
                std::cout << id << ", ";
            }
            std::cout << "\n";
            ROBOTS_NUM = IDS.size();
            std::cout << "Number of robots : " << ROBOTS_NUM << std::endl;
            std::cout << "Using home position of robot " << IDS[0] << std::endl;

            // -------------- Subs and Pubs ----------------
            gps_subs_.resize(ROBOTS_NUM);
            odom_subs_.resize(ROBOTS_NUM);
            odom_pubs_.resize(ROBOTS_NUM);

            // Origin is starting position of UAV0 for every UAV
            // home_sub_ = nh_.subscribe<geographic_msgs::GeoPointStamped>("/uav"+std::to_string(IDS[0])+"/mavros/global_position/gp_origin", 1, std::bind(&Supervisor::home_callback, this, std::placeholders::_1));
            home_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>("/uav"+std::to_string(IDS[0])+"/mavros/global_position/global", 1, std::bind(&Supervisor::home_callback, this, std::placeholders::_1));
            for (int i = 0; i < ROBOTS_NUM; ++i)
            {
                gps_subs_[i] = nh_.subscribe<sensor_msgs::NavSatFix>("/uav"+std::to_string(IDS[i])+"/mavros/global_position/global", 1, std::bind(&Supervisor::gps_callback, this, std::placeholders::_1, i));
                odom_subs_[i] = nh_.subscribe<nav_msgs::Odometry>("/uav"+std::to_string(IDS[i])+"/mavros/local_position/odom", 1, std::bind(&Supervisor::odom_callback, this, std::placeholders::_1, i));
                odom_pubs_[i] = nh_.advertise<nav_msgs::Odometry>("/supervisor/uav"+std::to_string(IDS[i])+"/odom", 10);
            }
            timer_ = nh_.createTimer(ros::Duration(0.01), std::bind(&Supervisor::timer_callback, this));



            // Init robot poses (each robot in a column)
            robots.resize(13, ROBOTS_NUM);       // x,y,z,rx,ry,rz,rw,vx,vy,vz,vrx,vry,vrz
            robots.setZero();
            robots.row(6).setOnes();            // consistent quaternion
            starting_poses.resize(3, ROBOTS_NUM);       // x,y,z
            starting_poses.setZero();
            robots_local.resize(13, ROBOTS_NUM);
            robots_local.setZero();
            robots_local.row(6).setOnes();            // consistent quaternion
            p_i.resize(7);
            p_i.setZero();
            p_i(6) = 1;               // consistent quaternion
            R_i.setZero();

            got_starting_poses.resize(ROBOTS_NUM, false);

            
            /*transform.setOrigin(tf::Vector3(robots(0, ROBOT_ID), 
                                    robots(1, ROBOT_ID), 
                                    robots(2, ROBOT_ID)));

            transform.setRotation(tf::Quaternion(robots(3, ROBOT_ID),
                                        robots(4, ROBOT_ID),
                                        robots(5, ROBOT_ID),
                                        robots(6, ROBOT_ID)));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "uav_"+std::to_string(ROBOT_ID)));
            */

            std::cout << "Init complete.\n";

        } 

        ~Supervisor()
        {
            std::cout << "Supervisor destroyer called\n";
        }

        void stop();
        void timer_callback();
        void emulate_vision();
        void home_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
        void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg, int id);
        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg, int id);

    private:
        std::vector<int> IDS = {0};
        int ROBOTS_NUM;
        bool got_home = false;

        // Global origin
        double origin_lat = 0.0;
        double origin_lon = 0.0;
        double origin_alt = 0.0;
        
        // Robots poses
        Eigen::MatrixXd robots;
        Eigen::MatrixXd starting_poses;
        Eigen::MatrixXd robots_local;
        Eigen::VectorXd p_i;                // UAV[i] pose
        Eigen::Matrix3d R_i;                // Rotation matrix 
        std::vector<bool> got_starting_poses;

        // ROS
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;
        ros::Timer timer_;
        ros::Subscriber home_sub_;
        std::vector<ros::Subscriber> gps_subs_;
        std::vector<ros::Subscriber> odom_subs_;
        std::vector<ros::Publisher> odom_pubs_;

        tf2_ros::TransformBroadcaster br;


};

void Supervisor::stop()
{
    ros::Duration(0.1).sleep();
    ros::shutdown();
}

// void Supervisor::home_callback(const geographic_msgs::GeoPointStamped::ConstPtr& msg)
// {
//     origin_lat = msg->position.latitude;
//     origin_lon = msg->position.longitude;
//     origin_alt = msg->position.altitude;
//     got_home = true;
// }

void Supervisor::home_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    if (!got_home) {
        origin_lat = msg->latitude;
        origin_lon = msg->longitude;
        origin_alt = msg->altitude;
        got_home = true;
    }
}


/* ---------- USE GPS FOR POSITION, ODOM FOR ORIENTATION AND VELOCITY ------*/
void Supervisor::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg, int id)
{
    robots.col(id).head(3) = global_to_local(msg->latitude, msg->longitude, msg->altitude,
                                            origin_lat, origin_lon, origin_alt);
    if(!got_starting_poses[id]) {
        starting_poses.col(id) = robots.col(id).head(3);
        std::cout << "Starting position of robot " << IDS[id] << ": " << starting_poses.col(id).transpose() << std::endl;
        got_starting_poses[id] = true;
    }
    
}

void Supervisor::odom_callback(const nav_msgs::Odometry::ConstPtr& msg, int id)
{
    robots(3, id) = msg->pose.pose.orientation.x;
    robots(4, id) = msg->pose.pose.orientation.y;
    robots(5, id) = msg->pose.pose.orientation.z;
    robots(6, id) = msg->pose.pose.orientation.w;
    robots(7, id) = msg->twist.twist.linear.x;
    robots(8, id) = msg->twist.twist.linear.y;
    robots(9, id) = msg->twist.twist.linear.z;
    robots(10, id) = msg->twist.twist.angular.x;
    robots(11, id) = msg->twist.twist.angular.y;
    robots(12, id) = msg->twist.twist.angular.z;

    robots_local(0, id) = msg->pose.pose.position.x;
    robots_local(1, id) = msg->pose.pose.position.y;
    robots_local(2, id) = msg->pose.pose.position.z;
    robots_local(3, id) = msg->pose.pose.orientation.x;
    robots_local(4, id) = msg->pose.pose.orientation.y;
    robots_local(5, id) = msg->pose.pose.orientation.z;
    robots_local(6, id) = msg->pose.pose.orientation.w;
    robots_local(7, id) = msg->twist.twist.linear.x;
    robots_local(8, id) = msg->twist.twist.linear.y;
    robots_local(9, id) = msg->twist.twist.linear.z;
    robots_local(10, id) = msg->twist.twist.angular.x;
    robots_local(11, id) = msg->twist.twist.angular.y;
    robots_local(12, id) = msg->twist.twist.angular.z;
}

/*
void Supervisor::emulate_vision()
{
    p_i = robots.col(ROBOT_ID);
    tf2::Quaternion q(p_i(3), p_i(4), p_i(5), p_i(6));
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    R_i(0, 0) = cos(yaw);
    R_i(0, 1) = sin(yaw);
    R_i(0, 2) = 0.0;
    R_i(1, 0) = -sin(yaw);
    R_i(1, 1) = cos(yaw);
    R_i(1, 2) = 0.0;
    R_i(2, 0) = 0.0;
    R_i(2, 1) = 0.0;
    R_i(2, 2) = 1.0;
    
    for (int i = 0; i < ROBOTS_NUM; ++i)
    {
        robots_local.col(i) = R_i * (robots.col(i).head(3) - p_i.head(3));
        // Simulate vision detection
        double dist = robots_local.col(i).norm();
        double angle = abs(atan2(robots_local(1, i), robots_local(0,i)));
        if (angle <= 0.5*ROBOT_FOV_rad && dist <= ROBOT_RANGE)
        {
            // Detected
            n_msg.poses[i].position.x = robots_local(0, i);
            n_msg.poses[i].position.y = robots_local(1, i);
            n_msg.poses[i].position.z = robots_local(2, i);

            if (i != ROBOT_ID)
            {
                geometry_msgs::Pose pose_msg;
                pose_msg.position.x = robots_local(0, i);
                pose_msg.position.y = robots_local(1, i);
                pose_msg.position.z = robots_local(2, i);
                target_pubs_[i].publish(pose_msg);

                // TF
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "uav_"+std::to_string(ROBOT_ID);
                transformStamped.child_frame_id = "uav_"+std::to_string(ROBOT_ID)+"/target_"+std::to_string(i);
                transformStamped.transform.translation.x = robots_local(0, i);
                transformStamped.transform.translation.y = robots_local(1, i);
                transformStamped.transform.translation.z = robots_local(2, i);
                transformStamped.transform.rotation.x = 0.0;
                transformStamped.transform.rotation.y = 0.0;
                transformStamped.transform.rotation.z = 0.0;
                transformStamped.transform.rotation.w = 1.0;
                br.sendTransform(transformStamped);
            }
        }
    }
    n_msg.header.stamp = ros::Time::now();
    neigh_pub_.publish(n_msg);
}
*/

void Supervisor::timer_callback()
{
    if (!got_home) {
        std::cout << "Waiting for home ...\n";
        return;
    }


    for (int i = 0; i < ROBOTS_NUM; ++i) {

        // Publish local odom data
        nav_msgs::Odometry msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        // msg.child_frame_id = "uav" + std::to_string(IDS[i])+"/base_link";
        msg.pose.pose.position.x = robots(0, i);   
        msg.pose.pose.position.y = robots(1, i);   
        msg.pose.pose.position.z = robots(2, i);
        msg.pose.pose.orientation.x = robots(3, i);   
        msg.pose.pose.orientation.y = robots(4, i);   
        msg.pose.pose.orientation.z = robots(5, i);   
        msg.pose.pose.orientation.w = robots(6, i);
        msg.twist.twist.linear.x = robots(7, i);
        msg.twist.twist.linear.y = robots(8, i);
        msg.twist.twist.linear.z = robots(9, i);
        msg.twist.twist.angular.x = robots(10, i);
        msg.twist.twist.angular.y = robots(11, i);
        msg.twist.twist.angular.z = robots(12, i);
        odom_pubs_[i].publish(msg);

        // Publish TF
        
        geometry_msgs::TransformStamped ts;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "map";
        ts.child_frame_id = "uav" + std::to_string(IDS[i]) + "/odom";
        ts.transform.translation.x = starting_poses(0, i);
        ts.transform.translation.y = starting_poses(1, i);
        ts.transform.translation.z = starting_poses(2, i);
        ts.transform.rotation.x = 0.0;
        ts.transform.rotation.y = 0.0;
        ts.transform.rotation.z = 0.0;
        ts.transform.rotation.w = 1.0;
        br.sendTransform(ts);



        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "uav" + std::to_string(IDS[i])+"/odom";
        ts.child_frame_id = "uav" + std::to_string(IDS[i])+"/base_link";
        ts.transform.translation.x = robots_local(0, i);
        ts.transform.translation.y = robots_local(1, i);
        ts.transform.translation.z = robots_local(2, i);
        ts.transform.rotation.x = robots_local(3, i);
        ts.transform.rotation.y = robots_local(4, i);
        ts.transform.rotation.z = robots_local(5, i);
        ts.transform.rotation.w = robots_local(6, i);

        br.sendTransform(ts);
    }
    
    

}





/*******************************************************************************
* Main function
*******************************************************************************/
//alternatively to a global variable to have access to the method you can make STATIC the class method interested, 
//but some class function may not be accessed: "this->" method cannot be used

void nodeobj_wrapper_function(int){
    ROS_WARN("signal handler function CALLED");
    node_shutdown_request = 1;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "supervisor_node", ros::init_options::NoSigintHandler);
    signal(SIGINT, nodeobj_wrapper_function);

    //Controller node_controller;
    auto node_controller = std::make_shared<Supervisor>();

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