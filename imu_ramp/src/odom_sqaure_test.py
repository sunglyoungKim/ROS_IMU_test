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
    self.vel_pub = None
    self.odometry_sub = None

    def odom_ros_sub(self, msg):
        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):
        self.vel_pub.publish(msg)

    def start_ros_node(self):

        # Create a Ros node
        rospy.init_node(self.node_name, anonymous = True, log_level=rospy.INFO)



    def PID(error_list, p, q, desire_x, vel_msg):

        error = p[0] - desire_x 
        error_list.append(error)    
        
        forward_c = 5.0

        if len(error_list) > 9:
            cur_error = error_list[-1]
            Ierror = np.sum(np.array(error_list))
            Derror = (error_list[-1] - error_list[1]) / 10

            input_signal = cur_error * P + Ierror * I + Derror * D
            

            vel_msg.linear.x = np.clip(np.array(abs(turning_input) * forward_c), 0, max_forward)
            

            turning_input = np.clip(np.array(turning_input), neg_limit, pos_limit )
            
            if pitch < 0:
                turning_input  = -1* turning_input 

            
            vel_msg.angular.z =  turning_input
            
            velocity_publisher.publish(vel_msg)


def callback(data, desire_x, desire_yaw):

    
    turning_input = 0.
    max_forward = 0.2
    pos_limit = 0.3 #for turning
    neg_limit = -0.3 # for turning
    
    
    seq = data.header.seq
    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs

    p = [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
   
    q = [data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z]

    euler_angle = tf.transformations.euler_from_quaternion(q)
    
    q_cov = data.pose.covariance
 
    yaw = math.degrees(euler_angle[0])
    pitch = math.degrees(euler_angle[1])
    roll = math.degrees(euler_angle[2])


    velocity_publisher = rospy.Publisher('cmd_vel_mux/input/navi/', Twist, queue_size=10)
    vel_msg = Twist()
    
    vel_msg.linear.x = 0.
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0.
    vel_msg.linear.z = 0.
    vel_msg.angular.x = 0.
    vel_msg.angular.y = 0.
    vel_msg.angular.z = 0.

    




    error_in_x = p[0] - desire_x 
    error_list.append(error_in_x)    
    
    forward_c = 5.0

    if len(error_list) > 9:
        cur_error = error_list[-1]
        Ierror = np.sum(np.array(error_list))
        Derror = (error_list[-1] - error_list[1]) / 10

        turning_input = cur_error * P + Ierror * I + Derror * D
        

        vel_msg.linear.x = np.clip(np.array(abs(turning_input) * forward_c), 0, max_forward)
        

        turning_input = np.clip(np.array(turning_input), neg_limit, pos_limit )
        
        if pitch < 0:
            turning_input  = -1* turning_input 

        
        vel_msg.angular.z =  turning_input
        
        velocity_publisher.publish(vel_msg)

def listener():
    rospy.init_node('q_ramp_listner', anonymous = True)    
    
    q_sub = message_filters.Subscriber('/q_cov_from_IMU', PoseWithCovarianceStamped)
    ramp_sub = message_filters.Subscriber('/ramp', ramp_d)
    
    ts = message_filters.TimeSynchronizer([q_sub, ramp_sub], 1 )
    ts.registerCallback(callback)
    
    rospy.spin()

if __name__ == '__main__':
    
    global error_list, P, I, D
    P = 0.2
    I = 0.1
    D = 0.1

    error_in_x = LimitedList(maxLen = 10) # limitied list
    error_in_yaw = LimitedList(maxLen = 10) # limitied list
    listener()
