#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Module to drive robot with a given acceleration profile. Should be
called directly from the main.py wrapper. Arguments are defined in
the AccControl class docstring below.

Questions? sherwinl@bu.edu, BU CODES Lab
"""
from __future__ import division
import rospy
import numpy as np
import std_msgs.msg
import geometry_msgs.msg
import tf
import config as cfg
import os
import sys
sys.path.append('../')

from lateral_control import lateral_control
from trials import eco_and, human_trial
from helper.trajectory import liveplot
from helper.logprint import logprint
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

hostname = os.uname()[1]
MIN_V = 0
MAX_V = 0.4
glob_cmd_vel = 0

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
      zero_hd (float): Heading (in deg) when robot is pointed at south wall
      delay (int): Time delay for robot motion begins (in sec)
      trial (str): Control scenario being trialed ('ECO-AND' or 'HUMAN')
    """
    def __init__(self, robot, accel, vel=0, zero_hd=0,
                 delay=0, trial=None):
        rospy.init_node('acc_controller')
        self.hz = 10
        self.rate = rospy.Rate(self.hz)

        # Control trial
        self.trial = trial

        # Publishers
        self.vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.mocap = rospy.Subscriber('/vrpn_client_node/' + robot + '/pose',
                                      PoseStamped,
                                      self.mocap_cb)

        # Message datatypes
        self.cmd_vel = Twist()
        self.mocap = PoseStamped()

        # Input arguments
        self.init_accel = accel
        self.accel = self.init_accel
        if vel != 0:
            self.vel = vel
        else:
            self.vel = glob_cmd_vel
        self.delay = delay * self.hz
        self.zero_hd = np.radians(zero_hd)

        # Calibration adjustment
        self.calibrate = 0

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
        init_yaw += np.radians(self.calibrate) # CALIBRATION EPSILON

        # Coordinate transform if 0 heading is not south-facing
        if self.zero_hd == 0:
            self.init_heading = init_yaw
        else:
            if (init_yaw - self.zero_hd) < -np.pi:
                self.init_heading = (init_yaw - self.zero_hd) % np.pi
            else:
                self.init_heading = init_yaw - self.zero_hd

    def mocap_cb(self, data):
        """
        Callback function for mocap data.
        """
        self.mocap = data

    def update_acc(self, accel):
        """
        Update acceleration state.
        """
        self.accel = accel

    def control(self):
        """
        Primary controller method.
        """
        global glob_cmd_vel
        self.timer = 0

        # Initialize arrays to store robot pose
        self.x = []
        self.y = []

        # Trajectory state
        prev_state = 0

        while not rospy.is_shutdown():
            self.timer += 1
            self.rate.sleep()

            if self.timer >= self.delay:

                # Current pose and orientation
                pose_x = self.mocap.pose.position.x
                pose_y = self.mocap.pose.position.y
                orient_z = self.mocap.pose.orientation.z
                orient_w = self.mocap.pose.orientation.w
                yaw = euler_from_quaternion((0,0,orient_z,orient_w))[2]
                yaw += np.radians(self.calibrate) # CALIBRATION EPSILON

                # Stop if robot exceeds floor boundaries
                if ((pose_x < cfg.X_LABLIM[0]) or (pose_x > cfg.X_LABLIM[1]) or
                        (pose_y < cfg.Y_LABLIM[0]) or (pose_y > cfg.Y_LABLIM[1])):
                    break

                # Save pose data for trajectory plot
                self.x.append(pose_x)
                self.y.append(pose_y)

                # Coordinate transform if 0 heading is not south-facing
                if self.zero_hd != 0:
                    if (yaw - self.zero_hd) < -np.pi:
                        yaw = (yaw - self.zero_hd) % np.pi
                    else:
                        yaw = yaw - self.zero_hd

                # Update acceleration based on trial
                if self.trial == 'ECO-AND':
                    eco_and(pose_y, self.timer, self.hz)
                    self.accel = cfg.accel
                elif self.trial == 'HUMAN':
                    human_trial(pose_y, self.timer, self.hz, self.vel)
                    self.accel = cfg.accel

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
                    else:
                        self.vel = 0

                # Zero out velocity if calculation is negative
                if self.vel < 0:
                    self.vel = 0

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
                logprint(self.timer,[self.init_pos_x,self.init_pos_y],
                         [pose_x,pose_y],[orient_z,orient_w],
                         np.degrees(yaw),ov_unit,
                         [self.cmd_vel.linear.x,
                         self.cmd_vel.angular.z])
