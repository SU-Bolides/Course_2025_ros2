#!/usr/bin/env python3

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This is a simple example of a Webots controller running a Python ROS node thanks to rospy.
The robot is publishing the value of its front distance sensor and receving motor commands (velocity).
"""

import rospy
from std_msgs.msg import Float64
from vehicle import Driver
from control_bolide.msg import SpeedDirection
import os


def callback(data):
    global velocity
    global message
    message = 'Received velocity value: ' + str(data.speed)
    velocity = data.speed


robot = Driver()
timeStep = int(robot.getBasicTimeStep())
velocity = 0
print('Initializing ROS: connecting to ' + os.environ['ROS_MASTER_URI'])
rospy.init_node('listener', anonymous=True)
print('Subscribing to "motor" topic')
rospy.Subscriber('/cmd_vel', SpeedDirection, callback)
print('Running the control loop')
while robot.step() != -1 and not rospy.is_shutdown():
    print("Velocity ", velocity)
    robot.setCruisingSpeed(6*velocity)