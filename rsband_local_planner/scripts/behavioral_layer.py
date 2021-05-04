#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import csv
import os
import rospkg 
import math
import numpy as np
from numpy import linalg as la
from tf.transformations import euler_from_quaternion, quaternion_from_euler


### This class will send the list of locations for the "mission", the "mission will be a list of goal locations for the F1Tenth"

class BehaviouralLayer:
    def __init__(self,racecar_name="racecar"):

        # move base uses an action server
        self.client= actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.current_goal = None
        self.xy_points = np.zeros((1,2))
        self.load_goals()
        self.tolerance = 1.0 
        self.planning_horizon = 2.0

        rospy.Subscriber(racecar_name+"/odom", Odometry, self.odom_callback, queue_size=1)


    def load_goals(self,path_file='track_porto_xy_sampled.csv'):
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        #get the path for this paackage
        package_path=rospack.get_path('rsband_local_planner')
        filename=os.path.sep.join([package_path,'waypoints',path_file])

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = np.asarray([float(point[0]) for point in path_points])
        self.path_points_y   = np.asarray([float(point[1]) for point in path_points])

        # list of xy pts 
        self.xy_points = np.hstack((self.path_points_x.reshape((-1,1)),self.path_points_y.reshape((-1,1)))).astype('double')

    def select_next_goal(self,x,y,yaw):
        curr_pos= np.asarray([x,y]).reshape((1,2))
        dist_arr = np.linalg.norm(self.xy_points-curr_pos,axis=-1)
        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
        goal_arr = np.where((dist_arr > self.planning_horizon))[0]
        
        # finding the goal point which is within the goal points 
        pts = self.xy_points[goal_arr]
        pts_infrontofcar=[]
        for idx in range(len(pts)): 
            v1 = pts[idx] - curr_pos
            #since the euler was specified in the order x,y,z the angle is wrt to x axis
            v2 = [np.cos(yaw), np.sin(yaw)]

            angle= self.find_angle(v1,v2)
            if angle < np.pi/2:
                pts_infrontofcar.append(pts[idx])

        pts_infrontofcar =np.asarray(pts_infrontofcar)
        dist_arr = np.linalg.norm(pts_infrontofcar-curr_pos,axis=-1)- self.planning_horizon
        
        # get the point closest to the lookahead distance
        idx = np.argmin(dist_arr)

        # goal point 
        goal_point = pts_infrontofcar[idx]
        return goal_point




    def send_goal(self,x=0.0,y=0.0):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # This line is required lol, it won't work otherwise
        goal.target_pose.pose.orientation.w = 1.0

        self.current_goal = goal
        for i in range(5):
            self.client.send_goal(goal)


    def odom_callback(self,data):
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
        #dist_arr = np.linalg.norm(self.xy_points-curr_pos,axis=-1)
        if(self.current_goal):
            dist_to_goal = np.linalg.norm([x-self.current_goal.target_pose.pose.position.x,y-self.current_goal.target_pose.pose.position.y])
            if(dist_to_goal<self.tolerance):
                rospy.logwarn("Goal Reached")
                new_goal = self.select_next_goal(x,y,yaw)
                rospy.logwarn("New Goal: {}, {}".format(new_goal[0],new_goal[1]))
                self.send_goal(new_goal[0],new_goal[1])


    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2).astype('double')
        sinang = la.norm(np.cross(v1, v2)).astype('double')
        return np.arctan2(sinang, cosang).astype('double')


if __name__ == '__main__':
    rospy.init_node('behavioural_layer')
    #get the arguments passed from the launch file
    #args = rospy.myargv()[1:]
    # get the racecar name so we know what to subscribe to
    #racecar_name=args[0]
    # get the path to the file containing the waypoints
    #waypoint_file=args[1]
    bl = BehaviouralLayer()
    rospy.sleep(3)
    bl.send_goal()
    # spin
    rospy.spin()