#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
import rospkg 

class pure_pursuit:

    def __init__(self,racecar_name='racecar',vesc_name='vesc'):

        # initialize class fields 
        self.racecar_name = racecar_name
        #self.waypoint_file = waypoint_file

        # pure pursuit parameters
        self.LOOKAHEAD_DISTANCE = 1.1#1.70 # meters
        
        # Distance from the 
        self.distance_from_rear_wheel_to_front_wheel = 0.5

        self.VELOCITY = 3.2 # m/s
        #self.read_waypoints()
        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher('/'+vesc_name+'/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=10)
        # Publisher for the goal point
        self.goal_pub = rospy.Publisher(racecar_name+'/goal_point', MarkerArray, queue_size="1")
        self.considered_pub= rospy.Publisher(racecar_name+'/considered_points', MarkerArray, queue_size="1")
        self.point_in_car_frame= rospy.Publisher(racecar_name+'/goal_point_car_frame', MarkerArray, queue_size="1")

        # Subscriber to vehicle position 
        rospy.Subscriber("move_base/RSBandPlannerROS/global_plan", Path, self.path_callback, queue_size=10)

        # path points 
        self.xy_points = []

        # Subscriber to vehicle position 
        rospy.Subscriber(racecar_name+"/odom", Odometry, self.callback, queue_size=10)

    # path callback message
    def path_callback(self,path_msg):
        points = []
        # the poses have more than x and y but initially let's just play with those
        for ps in path_msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            points.append([x,y])

        self.xy_points = np.asarray(points).reshape((-1,2))
        #print(self.xy_points.shape)
        self.visualize_point(points,self.considered_pub,count=100)

    
    # Visualize the path points
    def visualize_point(self,pts,publisher,frame='/map',r=1.0,g=0.0,b=1.0,count=100):
        # create a marker array
        markerArray = MarkerArray()

        idxs = np.random.randint(0,len(pts),min(count,len(pts)))
        for idx in idxs: 
            pt = pts[idx]

            x = float(pt[0])
            y = float(pt[1])
            
            marker = Marker()
            marker.id = idx
            marker.header.frame_id = frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0
            marker.lifetime = rospy.Duration(0.05)
            markerArray.markers.append(marker)
        publisher.publish(markerArray)


    # Input data is PoseStamped message from topic racecar_name/odom.
    # Runs pure pursuit and publishes velocity and steering angle.
    def callback(self,data):

        qx=data.pose.pose.orientation.x
        qy=data.pose.pose.orientation.y
        qz=data.pose.pose.orientation.z
        qw=data.pose.pose.orientation.w

        quaternion = (qx,qy,qz,qw)
        euler = euler_from_quaternion(quaternion)
        yaw   = np.double(euler[2])

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        ## finding the distance of each way point from the current position 
        curr_pos= np.asarray([x,y]).reshape((1,2))

        if(len(self.xy_points)>5):
            dist_arr = np.linalg.norm(self.xy_points-curr_pos,axis=-1)

            ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
            goal_arr = np.where((dist_arr > self.LOOKAHEAD_DISTANCE) & (dist_arr<self.LOOKAHEAD_DISTANCE+0.3))[0]
            
            # finding the goal point which is within the goal points 
            pts = self.xy_points[goal_arr]

            # get all points in front of the car, using the orientation 
            # and the angle between the vectors
            pts_infrontofcar=[]
            for idx in range(len(pts)): 
                v1 = pts[idx] - curr_pos
                #since the euler was specified in the order x,y,z the angle is wrt to x axis
                v2 = [np.cos(yaw), np.sin(yaw)]

                angle= self.find_angle(v1,v2)
                if angle < np.pi/2:
                    pts_infrontofcar.append(pts[idx])

            pts_infrontofcar =np.asarray(pts_infrontofcar)
            # compute new distances

            # if there are no more points in front of the car just stop
            try:
                dist_arr = np.linalg.norm(pts_infrontofcar-curr_pos,axis=-1)- self.LOOKAHEAD_DISTANCE
                
                # get the point closest to the lookahead distance
                idx = np.argmin(dist_arr)

                # goal point 
                goal_point = pts_infrontofcar[idx]
                self.visualize_point([goal_point],self.goal_pub)

                
            
                # transform it into the vehicle coordinates
                v1 = (goal_point - curr_pos)[0].astype('double')
                xgv = (v1[0] * np.cos(yaw)) + (v1[1] * np.sin(yaw))
                ygv = (-v1[0] * np.sin(yaw)) + (v1[1] * np.cos(yaw))

                vector = np.asarray([xgv,ygv])
                self.visualize_point([vector],self.point_in_car_frame,frame='racecar/chassis',r=0.0,g=1.0,b=0.0)
                
                # calculate the steering angle
                angle = math.atan2(ygv,xgv)
                self.const_speed(angle)
                #self.set_speed(angle)
            except:
                rospy.logwarn("No goal points ahead of the car")
    
    # # USE THIS FUNCTION IF CHANGEABLE SPEED IS NEEDED
    # def set_speed(self,angle):

    #     msg = drive_param()
        
    #     angle = abs(angle)
    #     if(angle <0.01):
    #         speed = 3.0#11.5
    #     elif(angle<0.0336332):
    #         speed = 2.7#11.1
    #     elif(angle < 0.0872665):
    #         speed = 2.4#7.6
    #     elif(angle<0.1309):
    #         speed = 2.3#6.5 
    #     elif(angle < 0.174533):
    #         speed = 2.2#6.0
    #     elif(angle < 0.261799):
    #         speed = 2.1#5.5
    #     elif(angle < 0.349066):
    #         speed = 1.5#3.2
    #     elif(angle < 0.436332):
    #         speed = 1.3#5.1
    #     else:
    #         print("more than 25 degrees",angle)
    #         speed = 1.0
    #     msg.velocity = speed
    #     msg.header.stamp = rospy.Time.now()
    #     self.pub.publish(msg)


    # USE THIS FUNCTION IF CONSTANT SPEED IS NEEDED
    def const_speed(self,angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.racecar_name+"/base_link"
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = np.clip(angle,-0.61,0.61)
        msg.drive.speed = 0.5
        self.pub.publish(msg)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2).astype('double')
        sinang = la.norm(np.cross(v1, v2)).astype('double')
        return np.arctan2(sinang, cosang).astype('double')


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    C = pure_pursuit()  
    # spin
    rospy.spin()