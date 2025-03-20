#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion

class MoveBarState(EventState):
    '''
    Sends a fixed navigation goal to move_base.

    The robot moves to the fixed position (2.33, -0.90, 0) with orientation 0 deg
    (quaternion: 0,0,0,1). Once the goal is reached, it returns the outcome 'Bar'.

    Outcomes:
        Bar      The robot has reached the bar position.
        failed   An error occurred during sending or execution of the goal.
    '''
    
    def __init__(self):
        super(MoveBarState, self).__init__(outcomes=['Bar', 'failed'])
        self._action_topic = 'move_base'
        self._client = ProxyActionClient({self._action_topic: MoveBaseAction})
        self.server_url = None  # Not used in this state.
        self._error = False

    def on_enter(self, userdata):
        # Create fixed goal for the bar with updated coordinates: (2.33, -0.90, 0) and 0° orientation.
        goal = MoveBaseGoal()
        fixed_pose = PoseStamped()
        fixed_pose.header.stamp = rospy.Time.now()
        fixed_pose.header.frame_id = "map"
        fixed_pose.pose.position = Point(2.33, -0.90, 0.0)
        fixed_pose.pose.orientation.x = 0.0
        fixed_pose.pose.orientation.y = 0.0
        fixed_pose.pose.orientation.z = 0.0
        fixed_pose.pose.orientation.w = 1.0
        goal.target_pose = fixed_pose
        
        try:
            self._client.send_goal(self._action_topic, goal)
            Logger.loginfo("Bar goal sent: %s" % str(fixed_pose))
        except Exception as e:
            Logger.logwarn("Failed to send Bar goal: %s" % str(e))
            self._error = True

    def execute(self, userdata):
        if self._error:
            return 'failed'
        # When the goal is reached, return outcome immediately.
        if self._client.has_result(self._action_topic):
            Logger.loginfo("Bar goal reached.")
            return 'Bar'
        return None

    def on_exit(self, userdata):
        # Cancel active goal if state is exited prematurely.
        if not self._client.has_result(self._action_topic):
            self._client.cancel(self._action_topic)
            Logger.loginfo("Bar goal cancelled.")

