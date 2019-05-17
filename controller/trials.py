#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Module to compute acceleration given specified trial. Called directly
from the controller.acceleration module.

Questions? sherwinl@bu.edu, BU CODES Lab
"""
from __future__ import division
import numpy as np
import config as cfg
from helper.scale import mcity_to_lab, lab_to_mcity
from controller import eco_and as ea

def calc_opt_acc(t0_s,tf_s):
    """
    Formulates optimization problem and calls eco_and.py.

    Args:
      t0_s (float): Sampling start time
      tf_s (float): Sampling end time
    """

    # Compute sampled velocity
    v0 = lab_to_mcity(1) /(tf_s-t0_s)

    # Optimization problem starting points
    t0 = cfg.T_SGR
    a0 = 0
    b0 = 1.5

    # Solve optimal control problem
    opt = ea.OptimalAcc(v0, cfg.TG, cfg.TR, cfg.D0, cfg.SGR, cfg.L, t0, a0, b0)
    u_star = opt.solve().x
    return (u_star)

def eco_and(y, timer, hz):
    """
    ECO-AND acceleration control.

    Args:
      y (float): Start of control environment
      timer (float): Experiment timer
      hz (int): ROS frequency
    """

    if np.abs(y-(cfg.y0+1)) < cfg.e and cfg.sample is False:
        print('Start velocity sampling...')
        cfg.t0_s = timer / hz
        cfg.sample = True
    elif np.abs(y-cfg.y0) < cfg.e and cfg.crossed is False:
        print('End velocity sampling, entering control environment.')
        cfg.tf_s = timer / hz
        u = calc_opt_acc(cfg.t0_s,cfg.tf_s)
        cfg.a_star = u[1]
        cfg.b_star = u[2]
        cfg.crossed = True
    elif cfg.crossed is True and cfg.finished is False:
        if np.abs(y-cfg.yf) < cfg.e:
            print('Leaving control environment.')
            cfg.accel = 0
            cfg.finished = True
            #print('ECO-AND trial complete in',cfg.t_trial,'seconds!')
        else:
            u_t = cfg.a_star*cfg.t_trial + cfg.b_star # Compute u(t)
            cfg.accel = mcity_to_lab(u_t) # Update acceleration
            print('accel:',cfg.accel,u_t)
        cfg.t_trial = (timer / hz) - cfg.tf_s # Update time
    elif cfg.finished is True:
        cfg.accel = 0

def human_trial(y, timer, hz, vel):
    """
    Human driver acceleration control.

    Args:
      y (float): Start of control environment
      timer (float): Experiment timer
      hz (int): ROS frequency
      vel (float): Previous commanded velocity
    """

    # Negative if light green, positive if red
    t_light_bin = np.sign(np.sin(2*np.pi*(cfg.t_trial -
                          ((cfg.T*(cfg.D0+1))/(2*abs(cfg.SGR-2))))/cfg.T +
                          3*np.pi/2 - np.pi*cfg.TG/cfg.T) -
                          np.sin(np.pi*cfg.TG/cfg.T - np.pi/2))

    # Calculate current light
    if t_light_bin > 0:
        t_light = 0
    else:
        t_light = 1

    if np.abs(y-cfg.y0) < cfg.e and cfg.crossed is False:
        print('Entering control environment.')
        cfg.t0 = timer / hz
        cfg.crossed = True
    elif cfg.crossed is True and cfg.finished is False:
        if t_light == 1:  # Green light
            if round(vel,2) < mcity_to_lab(cfg.vM):
                cfg.accel = mcity_to_lab(cfg.aM)
            elif np.abs(y-cfg.yf) < cfg.e:
                print('Leaving control environment.')
                cfg.accel = 0
                cfg.finished = True
            else:
                cfg.accel = 0
        elif t_light == 0:  # Red light
            if y < (cfg.yf+mcity_to_lab(14)):
                if y >= cfg.yf:
                    dist_rem = np.abs(y - cfg.yf)
                    cfg.accel = -(vel**2) / (2*dist_rem)
                elif np.abs(y-cfg.yf) < cfg.e:
                    print('Leaving control environment.')
                    cfg.accel = -vel
                    cfg.finished = True
                    #print('Human driver trial complete in',cfg.t_trial,'seconds!')
            elif y >= (cfg.yf+mcity_to_lab(14)):
                cfg.accel = 0
        #print('Trial timer:',round(t_trial))
        cfg.t_trial = (timer / hz) - cfg.t0
    elif cfg.finished is True:
        cfg.accel = -vel
