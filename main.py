#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Script to conduct CAV control trial. Run in the terminal with a
sys.argv argument specifying the trial type (either ECO-AND ("ECO-AND")
or human driver ("HUMAN"). Other environmental parameters can be
modified in the constants.

Example:
    $ python main.py 'ECO-AND'
    $ python main.py 'HUMAN'

Questions? sherwinl@bu.edu, BU CODES Lab
"""
import sys
import numpy as np
from controller import config as cfg
from controller import acceleration as acc
from helper.scale import mcity_to_lab, lab_to_mcity
from helper.trajectory import liveplot


# Environment constants
DELAY = 5  # Initial delay before robot moves
TG = 10  # Green interval
TR = 10  # Red interval
SGR = 1  # Starting light (green=1, red=0)
D0 = 0.5  # Fraction of time remaining of starting light interval
L = 70  # Road segment length
vM = 13  # Maximum velocity
vm = 2.78  # Minimum velocity
aM = 2.5  # Maximum acceleration
am = -2.9  # Minimum acceleration
T = TG+TR # Light cycle period
e = 0.05 # Start/end proximity threshold

# Starting light interval
if SGR == 1:
    T_SGR = TG
else:
    T_SGR = TR

# Road segment start/endpoints
y0 = -4.4
yf = y0 - mcity_to_lab(L)

if __name__== "__main__":
    # Initialize global variables
    cfg.init(thresh=e, road_len=L, ctrl_env_start=y0,
             ctrl_env_end=yf, max_vel=vM, min_vel=vm,
             max_acc=aM, min_acc=am, green=TG, red=TR,
             start_frac=D0, start_len=T_SGR,
             start_light=SGR)

    # Initialize instance of acceleration controller
    acc_obj = acc.AccControl(robot='create3',
                             accel=0,
                             vel=0.1,
                             zero_hd=0,
                             delay=10,
                             trial='ECO-AND')

    # Initialize control with constant velocity
    #acc_obj = acc.AccControl(robot=sys.argv[1],
    #                         accel=float(sys.argv[2]),
    #                         vel=float(sys.argv[3]),
    #                         zero_hd=float(sys.argv[4]),
    #                         delay=int(sys.argv[5]).
    #                         trial=sys.argv[6],)

    acc_obj.control()
    liveplot(acc_obj.x,acc_obj.y,
             cfg.X_LABLIM,cfg.Y_LABLIM,
             'Robot Trajectory')
