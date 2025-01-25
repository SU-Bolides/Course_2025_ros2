#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from vehicle import Driver
from controller import Lidar
from controller import GPS

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


#%% CLASS
class SimuLidarPublisher():

    def __init__(self, driver:Driver, bot_name:str="simu_bot") -> None:
        
        self.driver = driver
        
        # Publisher
        topic_name = "/raw_lidar_data"
        rospy.loginfo("Initializing lidar publisher on topic: " + topic_name)
        self.pub_scan = rospy.Publisher(topic_name, LaserScan, queue_size=10)
        self.pub_odom = rospy.Publisher("/gps/odom", Odometry, queue_size=10)

        self.basicTimeStep = int(self.driver.getBasicTimeStep())
        self.sensorTimeStep = 4 * self.basicTimeStep

        # Lidar initialization
        self.lidar_init()
    

    def lidar_init(self):
        """Initialize the lidar settings and the lidarScan message"""
        self.lidar = Lidar("RpLidarA2")
        self.lidar.enable(self.sensorTimeStep)
        self.lidar.enablePointCloud()

        self.lidarScan = LaserScan()
        self.lidarScan.header.stamp = rospy.Time.now()
        self.lidarScan.header.frame_id = "lidar_frame"
        self.lidarScan.angle_min = 0 # in radians
        self.lidarScan.angle_max = 2*np.pi
        self.lidarScan.angle_increment = 2*np.pi/1153
        self.lidarScan.time_increment = self.lidar.getSamplingPeriod() / 1000.
        self.lidarScan.scan_time = self.lidarScan.time_increment * 1153
        self.lidarScan.range_min = 0.17
        self.lidarScan.range_max = 12.0

        self.lidarScan.ranges = [0.] * 1153

        self.odometry = Odometry()

        self.gps = GPS("gps")
        self.gps.enable(self.sensorTimeStep)



    def treat_gps_data(self, gps_data):
        odom = Odometry()
        odom.header.frame_id = "lidar_frame"
        odom.pose.pose.position.x = gps_data[0]
        odom.pose.pose.position.y = gps_data[1]
        odom.pose.pose.position.z = gps_data[2]

        return odom

    
    def treat_lidar_data(self, lidar_data) -> list:
        """outputs 360 values, no inf values"""
        num_degrees = 1153
        num_bin = len(lidar_data)
        bin_size = num_bin / num_degrees

        # Create an array to store the resampled data
        resampled_data = np.zeros(num_degrees)

        # Resample the data
        for i in range(num_degrees):
            # Find the corresponding bin index
            bin_index = int(i / bin_size)
            
            # Calculate the average distance for the bin
            distance = lidar_data[bin_index]
            resampled_data[num_degrees - 1 - i] = distance if (distance not in [np.nan, np.inf]) else 0.
        
        return resampled_data


    def publish_lidar_data(self, *args):
        """Publishes the lidar data in the publisher topic"""
        # lidar
        self.lidarScan.ranges = self.treat_lidar_data( self.lidar.getRangeImage() )
        self.odometry = self.treat_gps_data(self.gps.getValues())
        self.lidarScan.header.stamp = rospy.Time.now()
        self.odometry.header.stamp = rospy.Time.now()
        self.pub_scan.publish(self.lidarScan)
        self.pub_odom.publish(self.odometry)
