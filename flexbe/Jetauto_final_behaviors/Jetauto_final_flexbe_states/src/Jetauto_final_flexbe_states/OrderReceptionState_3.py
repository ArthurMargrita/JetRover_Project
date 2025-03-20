#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from flexbe_core import EventState, Logger
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import urllib2
import urllib
import json

class OrderReceptionState_3(EventState):
    def __init__(self):
        super(OrderReceptionState_3, self).__init__(
            outcomes=['move', 'verif_2', 'erreur'],
            output_keys=['goal_pose_key', 'command', 'table']
        )
        self.server_url = 'http://192.168.149.71:5000'
        self._order_received = False
        self._order = None

    def log_order_list(self, prefix):
        url_all = self.server_url + '/get_all'
        try:
            response_all = urllib2.urlopen(url_all, timeout=1)
            data_all = response_all.read().decode('utf-8')
            try:
                order_list = json.loads(data_all)
            except Exception:
                order_list = data_all
            Logger.loginfo(u"%s - Contenu de la liste: %s" % (prefix, order_list))
        except urllib2.HTTPError as e:
            Logger.loginfo(u"%s - HTTP error %s" % (prefix, e.code))
        except Exception as e:
            Logger.loginfo(u"%s - Exception: %s" % (prefix, e))

    def on_enter(self, userdata):
        self._order_received = False
        self._order = None
        Logger.loginfo(u"Waiting for command...")
        self.log_order_list("on_enter")

    def execute(self, userdata):
        try:
            self.log_order_list("Before GET /get_first")
            url = self.server_url + '/get_first'
            Logger.loginfo(u"Retrieving command from URL: %s" % url)
            
            try:
                response = urllib2.urlopen(url, timeout=1)
            except urllib2.HTTPError as e:
                if e.code == 404:
                    Logger.loginfo(u"No command available (status 404)")
                    return 'verif_2'
                else:
                    raise

            data = response.read().decode('utf-8')
            order = json.loads(data)
            self.log_order_list("After JSON parse")

            if not order:
                return 'verif_2'

            if not self._order_received:
                self._order = order
                self._order_received = True

                if ';' in order:
                    parts = order.split(';')
                else:
                    parts = order.split()

                table_number = parts[0].strip()
                command_text = parts[1].strip() if len(parts) > 1 else ""

                userdata.command = command_text
                userdata.table = table_number

                goal_pose = PoseStamped()
                goal_pose.header.stamp = rospy.Time.now()
                goal_pose.header.frame_id = "map"
                
                table_positions = {
                    '1': (0.20, 1.35, 0.0, 0.0, 0.0, 0.7071, 0.7071),
                    '2': (-1.96, 1.46, 0.0, 0.0, 0.0, 0.7071, 0.7071),
                    '3': (-3.0, -1.3, 0.0, 0.0, 0.0, -0.7071, 0.7071),
                    '4': (0.33, -1.39, 0.0, 0.0, 0.0, -0.7071, 0.7071)
                }

                if table_number in table_positions:
                    x, y, z, qx, qy, qz, qw = table_positions[table_number]
                    goal_pose.pose.position = Point(x, y, z)
                    goal_pose.pose.orientation = Quaternion(qx, qy, qz, qw)
                else:
                    Logger.logwarn(u"Invalid table number, command removed")
                    try:
                        req = urllib2.Request(self.server_url + '/remove_first', data=urllib.urlencode({}))
                        req.get_method = lambda: 'POST'
                        urllib2.urlopen(req, timeout=1)
                    except Exception as e:
                        Logger.logwarn(u"Unable to remove first command: %s" % e)
                    return 'erreur'

                userdata.goal_pose_key = goal_pose
                Logger.loginfo(u"Command for table %s received. Goal: %s" % (table_number, str(goal_pose)))
                return 'move'
        except Exception as e:
            Logger.logwarn(u"Error retrieving command: {}".format(e))
        return None

    def on_exit(self, userdata):
        self._order_received = False
