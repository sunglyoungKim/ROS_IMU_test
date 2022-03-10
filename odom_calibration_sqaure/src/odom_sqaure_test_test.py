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

        self.node_name = "square_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel_mux/input/teleop"
        self.queue_size = 2
        self.pub_rate= 0.1
        
        # self.odometry_sub = None
        self.odom_position = None
        self.odom_orientation = None

    def odom_ros_sub(self, msg):
        self.odom_position = msg.pose.pose.position
        self.odom_orientation = msg.pose.pose.orientation

    def vel_ros_pub(self, msg):
        self.vel_pub.publish(msg)

    def start_ros_node(self):

        # Create a Ros node
        rospy.init_node(self.node_name, log_level=rospy.INFO)
        rospy.on_shutdown(self.stop_ros_node)

        self.odometry_sub = rospy.Subscriber(self.odom_sub_name, Odometry, callback=self.odom_ros_sub, queue_size= self.queue_size)
        self.vel_pub  = rospy.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

       

    def stop_ros_node(self):

        self.t_init = time.time()

        while time.time() - self.t_init < 1 and rospy.is_shutdown() == False:
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)


class squaremove(sending_signals):

    def __init__ (self):

        super(squaremove, self).__init__()
        self.max_forward = 0.1
        self.min_forward = -0.1
        self.max_angular = 0.5
        self.min_angular = -0.5   


    def get_x_y_position(self, pose):
    
        x = pose.x
        y = pose.y
    
      
        return x, y

    def get_yaw_angle(self, orientation):

        q_orig = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw = math.degrees(tf.transformations.euler_from_quaternion(q_orig)[2])


        return yaw
        

    def z_negative_90_rotation(self, orientation):


        q_rot = tf.transformations.quaternion_from_euler(0, 0, -1 * math.pi/2)
        q_orig = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

        q_new = tf.transformations.quaternion_multiply(q_rot , q_orig)
        yaw_new = math.degrees(tf.transformations.euler_from_quaternion(q_new)[2])

        return yaw_new 

    def z_positive_90_rotation(self, orientation):

        q_rot = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        q_orig = np.array([orientation.x, orientation.y, orientation.z, orientation.w])

        q_new = tf.transformations.quaternion_multiply(q_rot , q_orig)
        yaw_new = math.degrees(tf.transformations.euler_from_quaternion(q_new)[2])

        return yaw_new


    def PID(self, desire_x, error_list, P, I, D, forward = True):


        cur_error = error_list[-1]
        Ierror = np.sum(np.array(error_list))
        Derror = (error_list[-1] - error_list[1]) / 10

        _input = cur_error * P + Ierror * I + Derror * D

        
        if forward == True:       
            update_signal = np.clip(np.array(_input), self.min_forward, self.max_forward)
        else:
            update_signal = np.clip(np.array(_input), self.min_angular, self.max_angular)

        
        return update_signal



    def go_forward_goal(self, goal, x_y = True):

        if x_y == True:

            x, y = self.get_x_y_position(self.odom_position)
            cur_error = goal - x
            
            while abs(cur_error) > 0.01 and rospy.is_shutdown() == False:
                x, y = self.get_x_y_position(self.odom_position)
                cur_error = goal - x
                error_list.append(cur_error)    

                if len(error_list) > 9:

                    new_leanr_x_speed = self.PID(goal, error_list, P = 0.2, I = 0.1, D = 0.1)
                
                    msg = Twist()
                    msg.linear.x = new_leanr_x_speed
                    msg.angular.z = 0
                    self.vel_ros_pub(msg)

        else:

            x, y = self.get_x_y_position(self.odom_position)
            cur_error = goal - y
            
            while abs(cur_error) > 0.01 and rospy.is_shutdown() == False:
                x, y = self.get_x_y_position(self.odom_position)
                cur_error = goal - y
                error_list.append(cur_error)    

                if len(error_list) > 9:

                    new_leanr_x_speed = self.PID(goal, error_list, P = 0.2, I = 0.1, D = 0.1)
                
                    msg = Twist()
                    msg.linear.x = new_leanr_x_speed
                    msg.angular.z = 0
                    self.vel_ros_pub(msg)

            
    def turn_to_goal(self, goal):

        yaw = self.get_yaw_angle(self.odom_orientation)
        cur_error = goal - yaw
        
        while abs(cur_error) > 1.0 and rospy.is_shutdown() == False:
            yaw = self.get_yaw_angle(self.odom_orientation)
            cur_error = goal - yaw
            error_list.append(cur_error)    

            if len(error_list) > 9:

                new_angular_speed = self.PID(goal, error_list, P = 0.2, I = 0.1, D = 0.1, forward = False)
                
                msg = Twist()
                msg.linear.x = 0.0 
                msg.angular.z = new_angular_speed
                self.vel_ros_pub(msg)
                


    def move(self):

        while self.odom_position is None and rospy.is_shutdown() == False:
            time.sleep(0.5)
        
        self.go_forward_goal(1.53)
        new_yaw_goal = self.z_positive_90_rotation(self.odom_orientation)
        self.turn_to_goal(new_yaw_goal)
        
        self.go_forward_goal(1.53, x_y = False)
        new_yaw_goal = self.z_positive_90_rotation(self.odom_orientation)
        self.turn_to_goal(new_yaw_goal)
        
        self.go_forward_goal(0.0)
        new_yaw_goal = self.z_positive_90_rotation(self.odom_orientation)
        self.turn_to_goal(new_yaw_goal)

        # self.go_forward_goal(0.0, x_y = False)
        # new_yaw_goal = self.z_positive_90_rotation(self.odom_orientation)
        # self.turn_to_goal(new_yaw_goal)




if __name__ == '__main__':


    error_list = LimitedList(maxLen = 10) # limitied list

    r = squaremove()
    r.start_ros_node()
    r.move()
    
    # global error_list, P, I, D

    # error_in_x = LimitedList(maxLen = 10) # limitied list
    # error_in_yaw = LimitedList(maxLen = 10) # limitied list
    # listener()
