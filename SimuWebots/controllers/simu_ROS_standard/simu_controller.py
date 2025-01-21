#!/usr/bin/env python3
# -*- coding: utf-8 -*-

__author__ = "Eliot CHRISTON"
__date__ = "2023-01"


#%% IMPORTS
from vehicle import Driver

import numpy as np
import rospy
from std_msgs.msg import Bool
from control_bolide.msg import SpeedDirection


#%% CLASS
class SimuController():

    def __init__(self, driver:Driver, bot_name:str="simu_bot") -> None:
        
        self.driver = driver

        self.D = []
        self.speeds = []

        # Subscribers
        self.sub_param = rospy.Subscriber("/param_change_alert", Bool, self.get_max_params)
        self.sub_cmd_vel = rospy.Subscriber("/cmd_vel", SpeedDirection, self.set_command)
        self.sub_emergency_break = rospy.Subscriber("/emergency_break", Bool, self.emergency_break_callback)

        # Log info
        rospy.loginfo("Initializing controller for " + bot_name)
        rospy.loginfo("    Subscribing to /" + bot_name + "/cmd_vel")
        rospy.loginfo("    Subscribing to /" + bot_name + "/emergency_break")

        # Get the max speed and steering angle from the parameter server
        self.get_max_params()
        self.set_command(SpeedDirection(0., 0.))
        self.driver.setGear(1)

        # Initialize the speed, steering angle, and emergency break
        self.speed = 0.
        self.direction = 0.
        self.emergency_break = False

    def get_max_params(self, value = True) :
        self.maxSpeed = rospy.get_param('/simulation_max_speed', default = 18) / 3.6
        self.maxSteeringAngle = rospy.get_param('/simulation_max_angle', default = 30) * np.pi/180

    def cmd_vel_callback(self, msg:SpeedDirection):
        """Callback for the cmd_vel topic"""
        if not self.emergency_break:
            self.set_command(msg)

    def emergency_break_callback(self, msg:Bool):
        """Callback for the emergency_break topic"""
        if msg.data:
            self.set_command(SpeedDirection(0., 0.))
            self.emergency_break = True
        else:
            self.emergency_break = False

    def set_command(self, msg:SpeedDirection):
        """Set the speed and steering angle of the vehicle"""
        speed_command = msg.speed
        steeringAngle_command = msg.direction
        # make sure the commands are between -1 and 1
        
        # set the breaks if the speed command is 2
        if speed_command == 2:
            speed_command = 0

        assert speed_command >= -1 and speed_command <= 1, "Speed command must be between -1 and 1 (or 2 for breaks)"
        assert steeringAngle_command >= -1 and steeringAngle_command <= 1, "Steering angle command must be between -1 and 1"
        
        speed_command = msg.speed

        # convert the commands to the correct range
        self.speed = self.driver.getCurrentSpeed()
        # print("Speed ", self.driver.getCurrentSpeed())
        # print("RPM ", self.driver.getRpm())
        self.direction = steeringAngle_command * 0.4

        # self.speeds.append(self.driver.getCurrentSpeed())
        # self.D.append(speed_command)

        # np.savetxt("data.txt", np.column_stack((self.speeds, self.D)))

        speed_command = speed_command * 7

        print("Speedcommand ", speed_command)

        # apply the commands
        self.driver.setCruisingSpeed(speed_command)
        self.driver.setSteeringAngle(self.direction)
