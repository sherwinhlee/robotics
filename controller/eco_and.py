#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Module to compute optimal acceleration profile of CAV entering
a control environment. Arguments specified in class docstring
below.

Questions? sherwinl@bu.edu, BU CODES Lab
"""
from __future__ import division
from scipy.optimize import minimize
import config as cfg
import numpy as np

class OptimalAcc():
    """
    Optimization problem class.

    Args:
      v0 (float): Initial velocity
      tg (int): Green interval
      tr (int): Red interval
      D0 (float): Fraction of time remaining of starting light interval
      sgr (int): Starting light (green=1, red=0)
      l (int): Road segment length
      t0 (float): Optimization start parameter tau
      a0 (float): Optimization start parameter a
      b0 (float): Optimization start parameter b
      rho_t (float): Obj function param #1 (see Meng and Cassandras, 2019)
      rho_u (float): Obj function param #2 (see Meng and Cassandras, 2019)
      robot (str): Name of robot
      accel (float): Input acceleration in m/s^2
      vel (float): Input velocity in m/s (for constant speed)
      zero_hd (float): Heading (in deg) when robot is pointed at south wall
      delay (int): Time delay for robot motion begins (in sec)
      trial (str): Control scenario being trialed ('ECO-AND' or 'HUMAN')
    """
    def __init__(self, v0, tg, tr, D0, sgr, l, t0, a0, b0,
                       rho_t=0.5, rho_u=0.5):

        # Problem constants
        self.v0 = v0
        self.D0 = D0
        self.sgr = sgr
        self.l = l
        self.T = tg + tr
        self.D = tg / self.T
        self.t0 = t0
        self.a0 = a0
        self.b0 = b0
        self.rho_t = rho_t
        self.rho_u = rho_u
        self.vM = cfg.vM
        self.vm = cfg.vm
        self.aM = cfg.aM
        self.am = cfg.am

    def cost_fn(self, x):
        """
        Objective function.
        """

        # Acceleration profile variables
        t = x[:1]
        a = x[1:2]
        b = x[2:3]

        # Objective function
        f = self.rho_t*t + self.rho_u*(a**2*t**3/3 +
                                       a*b*t**2 +
                                       b**2*t)
        return f

    def solve(self):
        """
        Solve optimal control problem.
        """

        # Inequality constraints
        ineq = ({'type':'ineq',
             'fun':lambda x: np.array([self.aM - x[2],
                                       x[2]-self.am,
                                       x[0],
                                       self.vM - (self.v0 + x[2]*x[0] + 0.5*x[1]*x[0]**2),
                                       self.v0 + x[2]*x[0] + 0.5*x[1]*x[0]**2 - self.vm,
                                       np.sin(np.pi*self.D - np.pi/2)-
                                       np.sin(2*np.pi*(x[0] -
                                       ((self.T*(self.D0+1))/(2*abs(self.sgr-2))))/self.T +
                                       3*np.pi/2 - np.pi*self.D)])})

        # Equality constraints
        eq = ({'type':'eq',
               'fun':lambda x: np.array([x[1]*x[0] + x[2],
                                         self.v0*x[0] + 0.5*x[2]*x[0]**2 + x[1]*x[0]**3/6 - self.l])})

        # Starting points
        x0 = np.array([self.t0, self.a0, self.b0])

        # Solve optimization problem
        res = minimize(self.cost_fn,
                       x0=x0,
                       method='SLSQP',
                       constraints=[ineq,eq],
                       options={'disp': True})

        return res

if __name__== "__main__":
    vM = 13  # Maximum velocity
    vm = 2.78  # Minimum velocity
    aM = 2.5  # Maximum acceleration
    am = -2.9  # Minimum acceleration
    v0 = 3.69  # Initial velocity
    TG = 10  # Green light interval
    TR = 10  # Red light interval
    D0 = 0.5  # Fraction of time remaining of starting light interval
    SGR = 1  # Starting light (green=1, red=0)
    L = 70  # Road segment length
    t0 = 10  # Optimization start parameter tau
    a0 = 0  # Optimization start parameter a
    b0 = 1.5  # Optimization start parameter b

    # Initialize global variables
    cfg.init(thresh=None, road_len=L, ctrl_env_start=None,
             ctrl_env_end=None, max_vel=vM, min_vel=vm,
             max_acc=aM, min_acc=am, green=TG, red=TR,
             start_frac=D0, start_len=None, start_light=SGR)

    # Create instance of optimization problem class
    obj = OptimalAcc(v0, TG, TR, D0, SGR, L, t0, a0, b0)

    # Solve problem and return optimal acceleration profile
    u_t = obj.solve().x
    print(u_t[0])
    print(u_t[1])
    print(u_t[2])
