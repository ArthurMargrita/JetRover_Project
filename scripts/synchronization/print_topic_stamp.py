#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_ros import Buffer, TransformListener

def transform_callback():
    try:
        # Transformation odom -> base_footprint
        transform_odom_base_footprint = tf_buffer.lookup_transform('base_footprint', 'odom', rospy.Time(0))
        rospy.loginfo("Transform odom -> base_footprint Timestamp: %s" % transform_odom_base_footprint.header.stamp)

        # Transformation base_footprint -> base_link
        transform_base_footprint_base_link = tf_buffer.lookup_transform('base_link', 'base_footprint', rospy.Time(0))
        rospy.loginfo("Transform base_footprint -> base_link Timestamp: %s" % transform_base_footprint_base_link.header.stamp)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform not available: %s" % e)

if __name__ == '__main__':
    rospy.init_node('timestamp_transform_display')

    # Setup tf buffer and listener
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer)

    # Loop to check the transforms
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        transform_callback()
        rate.sleep()

