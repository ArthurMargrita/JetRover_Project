#!/usr/bin/env python
# encoding: utf-8
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import message_filters  

def callback(scan_msg, odom_msg):
    #rospy.loginfo("Messages synchronisés !")
    #rospy.loginfo("Scan timestamp: %f", scan_msg.header.stamp.to_sec())
    #.loginfo("Odom timestamp: %f", odom_msg.header.stamp.to_sec())
    
    sync_scan_pub.publish(scan_msg)
    sync_odom_pub.publish(odom_msg)

if __name__ == "__main__":
    rospy.init_node('sync_scan_odom')

    scan_sub = message_filters.Subscriber('/scan', LaserScan)
    odom_sub = message_filters.Subscriber('/odom', Odometry)

    sync_scan_pub = rospy.Publisher('/sync_scan', LaserScan, queue_size=10)
    sync_odom_pub = rospy.Publisher('/sync_odom', Odometry, queue_size=10)

    # Synchronisation plus tolérante
    ats = message_filters.ApproximateTimeSynchronizer(
        [scan_sub, odom_sub], queue_size=100, slop=1.5)
    ats.registerCallback(callback)

    rospy.loginfo("Noeud de synchronisation lancé...")
    rospy.spin()
