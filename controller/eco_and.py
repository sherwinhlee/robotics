"""
Class to compute optimal acceleration profile with the ECO-AND
control scenario. Primarily meant to be called by another
control script for real-time computation of optimization problem.

Questions? sherwinl@bu.edu, BU CODES Lab
"""

import numpy as np
from scipy.optimize import minimize

class OptimalAcc():
    def __init__(self, v0, tg, tr, D0, sgr, l, t0, a0, b0, 
                       rho_t=0.5, rho_u=0.5, vM=15, vm=2.78, aM=2.5, am=-2.9):
                       
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
        self.vM = vM
        self.vm = vm
        self.aM = aM
        self.am = am
    
    def cost_fn(self, x):
    
        # Acceleration profile variables
        t = x[:1]
        a = x[1:2]
        b = x[2:3]
        
        # Objective function
        f = self.rho_t*x[:1]+self.rho_u*(a**2*t**3/3 +
                                         a*b*t**2 +
                                         b**2*t)
        return f
    
    def solve(self):
    
        # Inequality constraints
        ineq = ({'type':'ineq',
             'fun':lambda x: np.array([self.aM - x[2],
                                       x[2]-self.am,
                                       x[0],
                                       self.vM - (self.v0 + x[2]*x[0] + 0.5*x[1]*x[0]**2),
                                       self.v0 + x[2]*x[0] + 0.5*x[1]*x[0]**2 - self.vm,
                                       np.sin(np.pi*self.D - np.pi/2)-
                                       np.sin(2*np.pi*(x[0] -((self.D0*self.T)/abs(self.sgr-2)))/self.T + 3*np.pi/2 - np.pi*self.D)])})
                                       
        # Equality constraints
        eq = ({'type':'eq',
               'fun':lambda x: np.array([x[1]*x[0] + x[2],
                                         self.v0*x[0] + 0.5*x[2]*x[0]**2 + x[1]*x[0]**3/6 - self.l])})

        # Starting points
        x0 = np.array([self.t0, self.a0, self.b0])
        
        # Solve optimization problem
        res = minimize(self.cost_fn, 
                       x0=x0,
                       constraints=[ineq,eq],
                       options={'disp': True})
        
        return res

if __name__== "__main__":
    v0 = 1
    tg = 20
    tr = 20
    D0 = 0.5
    sgr = 1
    l = 70
    t0 = 10
    a0 = -0.1
    b0 = 1.5
    
    # Create instance of optimization problem class
    obj = OptimalAcc(v0, tg, tr, D0, sgr, l, t0, a0, b0)
    
    # Solve problem and return optimal acceleration profile
    u_t = obj.solve().x
