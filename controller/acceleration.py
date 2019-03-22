#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Module to drive robot with a given acceleration profile. Can be 
imported from another controller script or run in the command line with
sys.argv arguments. Arguments are defined in the AccControl class 
docstring below.

Example: 
    $ python acceleration.py arg1 arg2 arg3 arg4 arg5
    
Questions? sherwinl@bu.edu, BU CODES Lab
"""
import rospy
import numpy as np
import std_msgs.msg
import geometry_msgs.msg
import tf
import os
import sys
sys.path.append('../')

from lateral_control import lateral_control
from helper.trajectory import liveplot
from helper.logprint import logprint
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

hostname = os.uname()[1]
MIN_V = 0
MAX_V = 0.35
glob_cmd_vel = 0

# Boundaries of BU Robotics Lab
X_LABLIM = [0,4]
Y_LABLIM = [-7,1]

def yaw_to_vec(angle):
    """
    Converts yaw component of euler angle to
    vector in Cartesian space. Assumes a 0
    angle heading is pointing toward the
    positive x-axis in the coordinate system.

    Args:
      angle (float): Angle of yaw orientation

    Returns:
      (x,y) (tuple): Cartesian vector
    """

    if abs(angle) > np.pi/2:
        x = -1
        y = x*np.tan(angle)
    elif abs(angle) < np.pi/2:
        x = 1
        y = x*np.tan(angle)
    elif angle == np.pi/2:
        x = 0
        y = 1
    else:
        x = 0
        y = -1

    return (x,y)

class AccControl():
    """
    Acceleration control class where each object instance is a discrete
    acceleration profile for the robot. 

    Args:
      robot (str): Name of robot
      accel (float): Input acceleration in m/s^2
      vel (float): Input velocity in m/s (for constant speed)
      coordsys (str): Cardinal direction of 0 heading (default is 's')
      delay (int): Time delay for robot motion begins (in sec)
    """
    def __init__(self, robot, accel, vel=None, coordsys='s', delay=0):
        rospy.init_node('acc_controller')
        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        # Publishers
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.mocap = rospy.Subscriber('/' + robot + '/pose',
                                      PoseStamped,
                                      self.mocap_cb)

        # Message datatypes
        self.cmd_vel = Twist()
        self.mocap = PoseStamped()

        # Input arguments
        self.init_accel = accel
        self.accel = self.init_accel
        if vel is not None:
            self.vel = vel
        else:
            self.vel = glob_cmd_vel
        self.delay = delay * self.hz
        self.coordsys = coordsys
        assert (self.coordsys in ['w','s']), "'Zero heading must be 'w' or 's'!"

        # Initialize mocap data
        self.initialize()

    def initialize(self):
        """
        Initial conditions.
        """
        self.rate.sleep()

        # Initial pose and orientation
        self.init_pos_x = self.mocap.pose.position.x
        self.init_pos_y = self.mocap.pose.position.y
        init_or_z = self.mocap.pose.orientation.z
        init_or_w = self.mocap.pose.orientation.w
        init_yaw = euler_from_quaternion((0,0,init_or_z,init_or_w))[2]

        # Coordinate transform if 0 heading is not south-facing
        if self.coordsys == 's':
            self.init_heading = init_yaw
        elif self.coordsys == 'w':
            if (init_yaw - np.pi/2) < -np.pi:
                self.init_heading = (init_yaw - np.pi/2) % np.pi
            else:
                self.init_heading = init_yaw - np.pi/2

    def mocap_cb(self, data):
        """
        Callback function for mocap data.
        """
        self.mocap = data

    def control(self):
        """
        Primarily controller method.
        """
        global glob_cmd_vel
        timer = 0

        # Initialize arrays to store robot pose
        self.x = []
        self.y = []

        # Trajectory state 
        prev_state = 0

        while not rospy.is_shutdown():
            timer += 1
            self.rate.sleep()
            
            if timer >= self.delay:
            
                # Current pose and orientation
                pose_x = self.mocap.pose.position.x
                pose_y = self.mocap.pose.position.y
                orient_z = self.mocap.pose.orientation.z
                orient_w = self.mocap.pose.orientation.w
                yaw = euler_from_quaternion((0,0,orient_z,orient_w))[2]

                # Stop if robot exceeds floor boundaries
                if ((pose_x < X_LABLIM[0]) or (pose_x > X_LABLIM[1]) or
                        (pose_y < Y_LABLIM[0]) or (pose_y > Y_LABLIM[1])):
                    break
                
                # Save pose data for trajectory plot
                self.x.append(pose_x)
                self.y.append(pose_y)

                # Coordinate transform if 0 heading is not south-facing
                if self.coordsys == 'w':
                    if (yaw - np.pi/2) < -np.pi:
                        yaw = (yaw - np.pi/2) % np.pi
                    else:
                        yaw = yaw - np.pi/2

                # Current orientation vector (info only)
                orient_vec = yaw_to_vec(yaw)
                ov_unit = orient_vec / np.linalg.norm(orient_vec)

                # Compute velocity
                if self.accel >= 0:
                    if MIN_V <= round(self.vel,2) < MAX_V:
                        self.vel += (self.accel / self.hz)
                else:
                    if MIN_V < round(self.vel,2) <= MAX_V:
                        self.vel += (self.accel / self.hz)

                # Adjust path trajectory with lateral control
                lc = lateral_control(prev_state, [self.init_pos_x,
                                     self.init_pos_y], [pose_x,pose_y], 
                                     self.init_heading, yaw, self.vel)
                                     
                prev_state = lc[1]
                self.cmd_vel.angular.z = lc[0]
                self.cmd_vel.linear.x = self.vel
                glob_cmd_vel = self.cmd_vel.linear.x
                self.vel_pub.publish(self.cmd_vel)
                
                # Print data to log file
                logprint(timer,[self.init_pos_x,self.init_pos_y],
                         [pose_x,pose_y],[orient_z,orient_w],
                         np.degrees(yaw),ov_unit,
                         [self.cmd_vel.linear.x,
                         self.cmd_vel.angular.z])

if __name__== "__main__":
    #obj = AccControl(robot='create1',accel=0.05,vel=None,coordsys='s',delay=10)
    obj = AccControl(robot=sys.argv[1],
                     accel=float(sys.argv[2]),
                     vel=float(sys.argv[3]),
                     coordsys=sys.argv[4],
                     delay=int(sys.argv[5]))
    obj.control()
    liveplot(obj.x,obj.y,X_LABLIM,Y_LABLIM,'Robot Trajectory')