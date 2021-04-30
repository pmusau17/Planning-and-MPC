#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
import numpy as np
import math


class TwistToAckermann:
    def __init__(self,racecar_name='racecar',vesc_name='vesc'):
        self.racecar_name = racecar_name
        self.ackermann_pub = rospy.Publisher('/'+vesc_name+'/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        self.twist_sub = rospy.Subscriber('cmd_vel', Twist,self.callback)

    def convert_trans_rot_vel_to_steering_angle(self, twist):
    
        # find the angle of the velocity command (this is the only way this thing worked for me)
        mag = np.linalg.norm([twist.linear.x,twist.linear.y]) 
        angle = math.atan2(twist.linear.y,twist.linear.x)
        speed = mag
        print(mag,angle)
        return angle, speed 

    def callback(self,twist_msg):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.racecar_name+"/base_link"
        angle,speed = self.convert_trans_rot_vel_to_steering_angle(twist_msg)
        msg.drive.speed = max(speed,0.3)
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = np.clip(angle,-0.61,0.61)
        self.ackermann_pub.publish(msg)


if __name__=="__main__":
    rospy.init_node('twist_to_ackermann', anonymous=True)
    TwistToAckermann()
    rospy.spin()




