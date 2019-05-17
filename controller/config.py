#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Configuration script to initialize global variables.
Arguments are environmental constants specific to
whatever environment being modeled.

Questions? sherwinl@bu.edu, BU CODES Lab
"""
def lab_dim():
    global X_LABLIM, Y_LABLIM
    X_LABLIM = [0,4]
    Y_LABLIM = [-8,2]

def init(thresh, road_len, ctrl_env_start,
         ctrl_env_end, max_vel, min_vel,
         max_acc, min_acc, green, red,
         start_frac, start_len, start_light):
    """
    Initialize global variables. Arguments are
    environmental constants. All other variables
    are those that need to be updated within
    the controller scripts.

    Args:
      thresh (float): Angle of yaw orientation
      road_len (int): Length of road segment
      ctrl_env_start (float): Start pos of control
                              environment (lab)
      ctrl_env_end (float): End pos of control
                            environment (lab)
      max_vel (float): Maximum velocity
      max_acc (float): Maximum acceleration
      green (int): Green light interval
      red (int): Red light interval
      start_frac (float): Fraction of start light
                          interval that has passed
      start_len (int): Time interval of start light
      start_light (int): Starting light (green = 1,
                                         red = 0)

    """
    lab_dim()

    global sample, crossed, finished, \
           t_trial, t0_s, tf_s, accel, \
           vel, a_star, b_star, e, L, \
           y0, yf, vM, vm, aM, am, TG, \
           TR, T, D0, T_SGR, SGR

    sample = False
    crossed = False
    finished = False
    t_trial = 0
    t0_s = 0
    tf_s = 0
    accel = 0
    vel = 0
    a_star = 0
    b_star = 0
    e = thresh
    L = road_len
    y0 = ctrl_env_start
    yf = ctrl_env_end
    vM = max_vel
    vm = min_vel
    aM = max_acc
    am = min_acc
    TG = green
    TR = red
    T = TG + TR
    D0 = start_frac
    T_SGR = start_len
    SGR = start_light
