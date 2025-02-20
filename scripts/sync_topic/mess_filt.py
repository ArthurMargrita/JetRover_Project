#!/usr/bin/env python
# encoding: utf-8
import rospy
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from subprocess import call

def callback(odom_msg, scan_msg):
    rospy.loginfo("Odometry timestamp: %s", odom_msg.header.stamp)
    rospy.loginfo("Scan timestamp: %s", scan_msg.header.stamp)


def listener():
    rospy.init_node('synchronization_node', anonymous=True)

    # Abonnir aux topics
    odom_sub = message_filters.Subscriber('/odom_raw', Odometry)
    scan_sub = message_filters.Subscriber('/scan', LaserScan)

    # Synchroniser les messages
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, scan_sub], queue_size=10, slop=1.0)  # Augmenter slop à 1.0


    ts.registerCallback(callback)

    # Lancer le nœud gmapping une fois que la synchronisation commence
    #rospy.loginfo("Launching gmapping...")
    #call(["rosrun", "gmapping", "slam_gmapping"])

    rospy.spin()

if __name__ == '__main__':
    listener()

