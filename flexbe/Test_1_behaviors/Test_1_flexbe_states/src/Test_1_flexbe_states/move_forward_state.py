#!/usr/bin/env python
import rospy
from flexbe_core import EventState
from geometry_msgs.msg import Twist

class MoveForwardFor5SecState(EventState):
    """
    Etat minimal pour faire avancer le robot pendant 5 secondes.
    
    -- speed    float    Vitesse linéaire en m/s

    <= done              Une fois les 5 secondes écoulées.
    """
    def __init__(self, speed):
        super(MoveForwardFor5SecState, self).__init__(outcomes=['done'])
        self.speed = speed
        self.duration = 5.0  # Durée de 5 secondes
        self._start_time = None
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def on_enter(self, userdata):
        self._start_time = rospy.Time.now()
        twist = Twist()
        twist.linear.x = self.speed
        self._pub.publish(twist)

    def execute(self, userdata):
        if rospy.Time.now() - self._start_time >= rospy.Duration(self.duration):
            return 'done'

    def on_exit(self, userdata):
        twist = Twist()  # Commande d'arrêt (tous les paramètres à 0)
        self._pub.publish(twist)

