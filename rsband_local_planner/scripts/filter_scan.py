#!/usr/bin/env python
from sensor_msgs.msg import LaserScan
import time
import rospy
import copy
import numpy as np
import math
import copy

"""The goal of this node is to filter the lidar data being used by the F1Tenth car as a sanity check
The sensormsgs/LaserScan message has the following fields
Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]
float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]
float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]
float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.
"""


class ProcessLidar:
    def __init__(self,racear_name='racecar'):
        self.pub= rospy.Publisher(racear_name+'/filtered_scan', LaserScan, queue_size="5")
        self.sub = rospy.Subscriber(racear_name+'/scan', LaserScan,self.lidar_callback, queue_size=5)


    def lidar_callback(self,data):
        new_scan=copy.copy(data)
        
        ranges=data.ranges
        ranges=np.nan_to_num(np.clip(np.asarray(ranges),0.0,9.99))
        new_scan.ranges=ranges
        #reset the time stamp
        new_scan.header.stamp=rospy.Time.now()
        self.pub.publish(new_scan)
        


if __name__=="__main__":
    rospy.init_node('process_lidar_data', anonymous=True)
    # Instantiate Lidar Node
    pl = ProcessLidar()
    rospy.spin()