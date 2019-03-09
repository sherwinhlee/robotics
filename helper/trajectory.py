# -*- coding: utf-8 -*-
"""
Helper function to plot trajectory of robots either 1) imported as a 
module to the controller scripts to plot live after an experiment, or
2) called directly from the command line to plot from an existing log
file.

Example: 
    $ python logprint.py logfile.txt "Robot Trajectory"
    
Questions? sherwinl@bu.edu, BU CODES Lab
"""
import os
import matplotlib.pyplot as plt
import sys

# Boundaries of BU Robotics Lab
X_LABLIM = [0,4]
Y_LABLIM = [-7,1]

def logplot(in_dir, fname, xlim, ylim, title):
    """
    Reads input log file, parses for x,y pose data and calls a plot
    function.

    Args:
      in_dir (str): Directory of log file
      fname (str): Filename of log file 
      xlim (arr[int]): x-axis limits
      ylim (arr[int]): y-axis limits
      title (str): Plot title
    """
    
    with open(in_dir + fname,'r') as logfile:
        lf_lines = logfile.readlines()
        
    traj_x = []
    traj_y = []

    for row in lf_lines:
        if row[:4] == 'pose':
            #print(float(row[10:-2]))
            tup = row[7:]
            sep_pos = tup.find(' , ')
            traj_x.append(float(tup[:sep_pos]))
            traj_y.append(float(tup[sep_pos+3:]))
            
    liveplot(traj_x, traj_y, xlim, ylim, title)
    
def liveplot(x, y, xlim, ylim, title):
    """
    Plot function called directly from controller file after an 
    experiment.
    
    Args:
      x, y (arr[float]): x, y points to plot
      xlim (arr[int]): x-axis limits
      ylim (arr[int]): y-axis limits
      title (str): Plot title
    """
    plt.plot(x,y,'b.')
    plt.xlim(xlim)
    plt.ylim(ylim)
    plt.xlabel('North-South Axis')
    plt.ylabel('East-West Axis')
    plt.title(title)
    plt.show()

if __name__== "__main__":
    parent = os.path.realpath('..')
    in_dir = parent + '\\logs\\'
    log = sys.argv[1]
    title = sys.argv[2]
    logplot(in_dir, log, 
            X_LABLIM, Y_LABLIM, 
            title)