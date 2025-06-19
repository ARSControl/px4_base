#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from pyproj import Proj
import tf
import tf_conversions


class Supervisor():
    def __init__(self):
        rospy.init_node("tf_broadcaster")
        self.ids = rospy.get_param("robots_ids", "[0, 1]")
        self.offset_x = rospy.get_param("~offset_x", 0.0)
        self.offset_y = rospy.get_param("~offset_y", 0.0)
        self.offset_z = rospy.get_param("~offset_z", 0.0)
        self.rotation = rospy.get_param("~rotation", 0.0)
        self.rotation *= np.pi / 180.0
        self.R = np.array([
                [np.cos(self.rotation), -np.sin(self.rotation), 0.0],
                [np.sin(self.rotation), np.cos(self.rotation),  0.0],
                [0.0,                   0.0,                    1.0]])
        T = np.eye(4)
        T[0:3, 0:3] = self.R
        T[0:3, 3] = np.array([self.offset_x, self.offset_y, 0.0])
        self.quat = tf_conversions.transformations.quaternion_from_matrix(T)
        print("Using offsets: ", self.offset_x, self.offset_y, self.offset_z)
        print("Rotation: ", self.rotation)
        print("Got IDS:")
        for id in self.ids:
            print(f"{id}, ")

        self.robots_num = len(self.ids)
        print("Number of drones: ", self.robots_num)
        print("Using home position of robot ", self.ids[0])

        self.ref_lat = None
        self.ref_lon = None
        self.ref_alt = None
        self.proj = None
        self.gps_data = [None] * self.robots_num
        self.local_odom = [None] * self.robots_num
        self.just_init = [True] * self.robots_num
        self.starting_positions = np.zeros((self.robots_num, 3))

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self.global_odom_pubs = [None] * self.robots_num

        # Define pubs/subs
        for i, id in enumerate(self.ids):
            gps_topic = f"/uav{id}/mavros/global_position/global"
            rospy.Subscriber(gps_topic, NavSatFix, self.gps_callback, callback_args=i)
            odom_topic = f"/uav{id}/mavros/local_position/odom"
            rospy.Subscriber(odom_topic, Odometry, self.odom_callback, callback_args=i)
            global_odom_topic = f"/supervisor/uav{id}/odom"
            self.global_odom_pubs[i] = rospy.Publisher(global_odom_topic, Odometry, queue_size=1)

    def gps_callback(self, msg, i):
        self.gps_data[i] = msg

        # set reference origin
        if self.proj is None and i == 0:
            self.ref_lat = msg.latitude
            self.ref_lon = msg.longitude
            self.ref_alt = msg.altitude
            self.proj = Proj(proj='aeqd', 
                            lat_0=self.ref_lat,
                            lon_0=self.ref_lon,
                            ellps='WGS84')
            rospy.loginfo(f"[TF] Reference origin set from uav {self.ids[i]}: ({self.ref_lat}, {self.ref_lon}, {self.ref_alt})")

        # send map -> local origin (just once)
        if self.proj is not None:
            if self.just_init[i]:
                x, y = self.proj(msg.longitude, msg.latitude)
                z = msg.altitude - self.ref_alt
                self.starting_positions[i, 0] = x + self.offset_x
                self.starting_positions[i, 1] = y + self.offset_y
                self.starting_positions[i, 2] = z + self.offset_z
                # self.starting_positions[i, 2] = 0.0
                print(f"Starting position of uav {self.ids[i]}: {self.starting_positions[i]}")
                self.just_init[i] = False


    def odom_callback(self, msg, i):
        print(f"Odom for robot {self.ids[i]}: {msg.pose.pose.position}")
        self.local_odom[i] = msg

    def run(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            t_now = rospy.Time.now()
            if self.proj is None:
                rate.sleep()
                continue
            
            for i in range(self.robots_num):
                # [TF] map -> local origin
                tf_origin = TransformStamped()
                tf_origin.header.stamp = t_now
                tf_origin.header.frame_id = f"map"
                tf_origin.child_frame_id = f"uav{self.ids[i]}/local_origin"
                tf_origin.transform.translation.x = self.starting_positions[i, 0]
                tf_origin.transform.translation.y = self.starting_positions[i, 1]
                tf_origin.transform.translation.z = self.starting_positions[i, 2]
                # self.quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.rotation)
                # tf_origin.transform.rotation.x = self.quat[0]
                # tf_origin.transform.rotation.y = self.quat[1]
                # tf_origin.transform.rotation.z = self.quat[2]
                # tf_origin.transform.rotation.w = self.quat[3]
                tf_origin.transform.rotation.w = 1
                self.tf_broadcaster.sendTransform(tf_origin)
                
                odom = self.local_odom[i]
                #print(f"Odom for robot {i}: {odom.pose.pose.position}")
                if odom is not None:
                    # [TF] local origin -> base link
                    tf_odom = TransformStamped()
                    tf_odom.header.stamp = t_now
                    tf_odom.header.frame_id = f"uav{self.ids[i]}/local_origin"
                    tf_odom.child_frame_id = f"uav{self.ids[i]}/base_link"

                    tf_odom.transform.translation.x = odom.pose.pose.position.x
                    tf_odom.transform.translation.y = odom.pose.pose.position.y
                    tf_odom.transform.translation.z = odom.pose.pose.position.z
    
                    tf_odom.transform.rotation = odom.pose.pose.orientation

                    self.tf_broadcaster.sendTransform(tf_odom)

                    # [Topic] map -> base link
                    odom_msg = Odometry()
                    odom_msg.header.stamp = t_now
                    odom_msg.header.frame_id = "map"
                    odom_msg.pose.pose.position.x = odom.pose.pose.position.x + self.starting_positions[i, 0]
                    odom_msg.pose.pose.position.y = odom.pose.pose.position.y + self.starting_positions[i, 1]
                    odom_msg.pose.pose.position.z = odom.pose.pose.position.z + self.starting_positions[i, 2]
                    odom_msg.pose.pose.orientation = odom.pose.pose.orientation
                    self.global_odom_pubs[i].publish(odom_msg)



            rate.sleep()

if __name__ == "__main__":
    node = Supervisor()
    node.run()
