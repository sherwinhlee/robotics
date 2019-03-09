# -*- coding: utf-8 -*-
"""
Helper function to print live data from ROS subscribers to a log file.
Primarily used for import to the controller scripts. Can also be tested
with arguments passed in directly from the command line.

Example: 
    $ python logprint.py arg1 arg2 arg3 arg4 arg5 arg6 arg7
    
Questions? sherwinl@bu.edu, BU CODES Lab
"""
import sys
import ast 

def logprint(timer,init_pos,pose,orient,yaw,ov_unit,cmd_vel):
    """
    Prints robot data to log file.

    Args:
      timer (int): ROS while loop counter
      init_pos (arr[float]): Initial robot position
      pose (arr[float]): Current robot position
      orient (arr[float]): Current robot orientation (quarternion)
      yaw (float): Current robot orientation (Euler angle in radians)
      ov_unit (arr[float]): Robot orientation in vector form
      cmd_vel (arr[float]): Commanded linear and angular velocities
    """
    print(timer)
    print('initial position:',
          round(init_pos[0],4),',',
          round(init_pos[1],4))
    print('pose:',
          round(pose[0],4),',',
          round(pose[1],4))
    print('orientation (q):',
          round(orient[0],4),',',
          round(orient[1],4))
    print('orientation (rad):',
          round(yaw,4))
    print('orientation vector:',
          round(ov_unit[0],4),',',
          round(ov_unit[1],4))
    print('cmd_vel:',
          cmd_vel[0],',',
          cmd_vel[1],'\n')
    
if __name__== "__main__":
    logprint(int(sys.argv[1]),
             ast.literal_eval(sys.argv[2]),
             ast.literal_eval(sys.argv[3]),
             ast.literal_eval(sys.argv[4]),
             float(sys.argv[5]),
             ast.literal_eval(sys.argv[6]),
             ast.literal_eval(sys.argv[7]))