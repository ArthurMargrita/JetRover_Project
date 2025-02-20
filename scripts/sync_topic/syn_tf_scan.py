#!/usr/bin/env python
# encoding: utf-8
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def scan_callback(scan_msg):
    global last_odom_msg
    if last_odom_msg is None:
        return

    br = tf.TransformBroadcaster()
    # Publier la transformation `odom → base_link` avec l'horodatage du LiDAR
    br.sendTransform(
        (last_odom_msg.pose.pose.position.x, last_odom_msg.pose.pose.position.y, 0),
        (last_odom_msg.pose.pose.orientation.x, last_odom_msg.pose.pose.orientation.y,
         last_odom_msg.pose.pose.orientation.z, last_odom_msg.pose.pose.orientation.w),
        scan_msg.header.stamp,  # Horodatage du LiDAR
        "base_link",
        "odom"
    )

def odom_callback(odom_msg):
    global last_odom_msg
    last_odom_msg = odom_msg  # Stocker le dernier message odom

rospy.init_node('tf_sync_node')
last_odom_msg = None

rospy.Subscriber('/scan_raw', LaserScan, scan_callback)
rospy.Subscriber('/odom', Odometry, odom_callback)

rospy.spin()
