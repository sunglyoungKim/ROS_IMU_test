#!/usr/bin/python

import rospy
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped

import tf
# import transforms3d as transfer
import math
from time import sleep, time
from math import sin, cos, tan, pi
import numpy as np

from collections import Counter


class LimitedList(list):

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


def callback(data):
    
    
    seq = data.header.seq
    secs = data.header.stamp.secs
    nsecs = data.header.stamp.nsecs
   
    q = [data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z]

    euler_angle = tf.transformations.euler_from_quaternion(q)
    
    q_cov = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ]
    
    q_list.append(q)
    euler_list.append(euler_angle)    
    
    accumulated_ramp = 'off'
       
    moving_avg_euler = np.array(euler_list).mean(axis = 0)
    moving_avg_q = np.array(q_list).mean(axis = 0)

    yaw = math.degrees(moving_avg_euler[0])
    pitch = math.degrees(moving_avg_euler[1])
    roll = math.degrees(moving_avg_euler[2])


    if (abs(roll) >  0.75) or (abs(pitch) > 0.75):
        ramp = 'on'
        ramp_list.append(ramp)

    else:
        ramp = 'off'
        ramp_list.append(ramp)


    if len(ramp_list) > 9:
    
        cn = Counter(ramp_list)


        for k,v in cn.most_common():
            break

        if (k == 'off') & (v > 5):
            accumulated_ramp = 'off'

        else:
            accumulated_ramp = 'on'
        
        
    pose_data = PoseWithCovarianceStamped()    
    
    pose_data.header.seq = seq
    pose_data.header.stamp.secs = secs
    pose_data.header.stamp.nsecs = nsecs
    
    pose_data.header.frame_id = "base_footprint"
    
    pose_data.pose.pose.position.x = 0.
    pose_data.pose.pose.position.y = 0.
    pose_data.pose.pose.position.z = 0.
    
    
    pose_data.pose.pose.orientation.w = moving_avg_q[0]
    pose_data.pose.pose.orientation.x = moving_avg_q[1]
    pose_data.pose.pose.orientation.y = moving_avg_q[2]
    pose_data.pose.pose.orientation.z = moving_avg_q[3]
    
    pose_data.pose.covariance = q_cov
        

    ramp_msg =  ramp_d()
    ramp_msg.ramp = accumulated_ramp
    ramp_msg.header.seq = seq
    ramp_msg.header.stamp.secs = secs
    ramp_msg.header.stamp.nsecs = nsecs

    
        
        
    # publish a topic
    pub_q_cov = rospy.Publisher('q_cov_from_IMU', PoseWithCovarianceStamped, queue_size=5)
    pub_ramp= rospy.Publisher('ramp', ramp_d, queue_size=5)
#     print(pose_data)
    pub_q_cov.publish(pose_data)
    pub_ramp.publish(ramp_msg)
#     print(rospy.get_rostime())
#     print(rospy.get_time())
#     print("")


#     pub_q = rospy.Publisher('q_from_IMU', Float32MultiArray, queue_size=5)  
#     pub_q.publish(data = moving_avg_q)

def listener():

    rospy.init_node('IMU_listner', anonymous = True)
#     rospy.Timer(rospy.Duration(1.0/60.0), ts.read_temperature_sensor_data)
    rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, callback)
#    rospy.Subscriber('/imu', Imu, callback)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(60)
#     while not rospy.is_shutdown():
#         hello_str = 'hello world %s' % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

    rospy.spin()
    

if __name__ == '__main__':
    global q_list,  euler_list
    
    q_list = LimitedList(maxLen = 10) # limitied list
    euler_list = LimitedList(maxLen = 10) # limitied list
    ramp_list = LimitedList(maxLen = 10) # limitied list
    listener()
