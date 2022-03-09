#!/usr/bin/python

import rospy
import message_filters
from geometry_msgs.msg import Twist, Pose, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry

import numpy as np
import math
import distutils
import tf
import time

class LimitedList(list):
    """
    truncated length of list
    """

    # Read-only
    @property
    def maxLen(self):
        return self._maxLen

    def __init__(self, *args, **kwargs):
        self._maxLen = kwargs.pop("maxLen")
        list.__init__(self, *args, **kwargs)

    def _truncate(self):
        """Called by various methods to reinforce the maximum length."""
        dif = len(self)-self._maxLen
        if dif > 0:
            self[:dif]=[]

    def append(self, x):
        list.append(self, x)
        self._truncate()

    def insert(self, *args):
        list.insert(self, *args)
        self._truncate()

    def extend(self, x):
        list.extend(self, x)
        self._truncate()

    def __setitem__(self, *args):
        list.__setitem__(self, *args)
        self._truncate()

    def __setslice__(self, *args):
        list.__setslice__(self, *args)
        self._truncate()
        

class sending_signals(object):
    """
    """

    def __init__(self):

        self.node_name = "sqaure_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel_mux/input/navi"
        self.queue_size = 10
        self.pub_rate= 0.1

    def odom_ros_sub(self, msg):
        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):
        self.vel_pub.publish(msg)

    def start_ros_node(self):

        # Create a Ros node
        rospy.init_node(self.node_name, anonymous = True, log_level=rospy.INFO)

        rospy.on_shutdown(self.stop_ros_node)

        self.odometry_sub = rospy.Subscriber(self.odom_sub_name, Odometry, callback=self.odom_ros_sub, queue_size= self.queue_size)
        self.vel_pub  = rospy.Publisher(self.vel_pub_name, Twist, queue_size=10)


    def stop_ros_node(self):

        self.t_init = time.time()

        while time.time() - self.t_init < 1 and not rospy.is_shutdown():
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

    def move(self):

        while not rospy.is_shutdown():
            time.sleep(1)


class squaremovevel(sending_signals):

    def __init__ (self):

        super(squaremovevel, self).__init__()

    def go_forward(self, duration, speed):

        self.t_init = time.time()

        while time.time() - self.t_init < duration and rospy.is_shutdown() == False:

            msg = Twist()
            msg.linear.x = speed
            msg.angular.z = 0
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)
            
    def turn(self, duration, ang_speed):

        self.t_init = time.time()

        while time.time() - self.t_init < duration and rospy.is_shutdown() == False:

            msg = Twist()
            msg.angular.x = 0
            msg.angular.z = ang_speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

    def move(self):
        self.go_forward(2, 0.2)
        self.turn(2, -0.5)



if __name__ == '__main__':


    r = squaremovevel()

    r.start_ros_node()
    r.move()
    
    # global error_list, P, I, D
    # P = 0.2
    # I = 0.1
    # D = 0.1

    # error_in_x = LimitedList(maxLen = 10) # limitied list
    # error_in_yaw = LimitedList(maxLen = 10) # limitied list
    # listener()
