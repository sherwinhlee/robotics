# -*- coding: utf-8 -*-
"""
Module to apply lateral control so robot follows a defined straight-
line trajectory based on pre-defined threshold of 3cm. Primary imported
into the main controller file. If the robot stays within the threshold
of 3cm, it is considered "on-course."

Trajectory states are as follows: 
    0: on-course, maintaining heading with no rotational velocity
    1: off-course, computes rotational velocity to correct heading
    2: on-course, but requires heading correction 

Questions? sherwinl@bu.edu, BU CODES Lab
"""
import numpy as np

def angular(diff_vec, diff_ang, vel):
    """
    Computes angular velocity of robot when entering back into 
    trajectory path threshold.

    Args:
      diff_vec (tuple): Delta vec to waypoint
      diff_ang (float): Angle between orient tangent line and diff_vec
      vec (float): Input speed of robot

    Returns:
      (float): Angular velocity to traverse arc
    """
    sweep_ang = 2*diff_ang
    arc_len = np.linalg.norm(diff_vec)*diff_ang*np.sin(diff_ang)**(-1)
    return (sweep_ang*vel)/arc_len
    
def heading(angle):
    """
    Computes cardinal direction based on heading/orientation. Assumes 
    robot heading is parallel to cartesian axes and 0 heading is south.

    Args:
      angle (float): Angle of yaw orientation

    Returns:
      (str): Cardinal direction
    """
    if -5*np.pi/9 <= angle <= -4*np.pi/9: # West
        return 'w'
    elif 4*np.pi/9 <= angle <= 5*np.pi/9: # East
        return 'e'
    elif np.abs(angle) <= np.pi/18: # South
        return 's'
    elif np.abs(angle) >= 17*np.pi/18: # North
        return 'n'
   
def lateral_control(prev_state, init_pos, pose, init_heading, yaw, vel):
    """
    Lateral control function to adjust robot heading based on
    trajectory position and orientation error.

    Args:
      prev_state (int): Previous trajectory state
      init_pos (arr[float]): Initial robot pose
      pose (arr[float]): Current robot pose
      init_heading (float): Initial robot orientation (radians)
      yaw (float): Current robot orientation (radians)
      vel (float): Current commanded linear velocity

    Returns:
      angular_z (float): Angular velocity for cmd_vel publisher
      prev_state (int): Update of trajectory state
    """
    
    # Default rotational velocity
    angular_z = 0
    
    # Position error
    del_x = init_pos[0] - pose[0]
    del_y = init_pos[1] - pose[1]

   # West heading
    if heading(init_heading) == 'w':
        if abs(del_x) <= 0.03: # On-course
            if prev_state == 1:
                print('STATE 2: On course: correcting heading...')
                del_y = del_x * np.tan((2*yaw-np.pi)/4)
                diff_ang = -(2*yaw+np.pi)/4
                ang_vel = angular((del_x,del_y),diff_ang,vel)
                arc_rad = vel / ang_vel
                angular_z = ang_vel
                prev_state = 2
            else:
                if abs(init_heading - yaw) <= np.pi/60:
                    print('STATE 0: Maintaining...')
                    angular_z = 0
                    prev_state = 0
                else:
                    if prev_state == 2:
                        print('STATE 2: On course: correcting heading...')
                        angular_z = vel / arc_rad
                        prev_state = 2
                    elif prev_state == 0:
                        print('STATE 0: Maintaining...')
                        angular_z = 0
                        prev_state = 0
        else: # Off-course
            print('STATE 1: Off course: correcting...')
            if (prev_state == 0) or (prev_state == 2):
                diff_ang = yaw + np.pi/2
                sweep_ang = 2*diff_ang
                arc_rad = vel / sweep_ang
                depart_ang = yaw
                return_ang = -(np.pi+depart_ang)
                angular_z = -sweep_ang
            else:
                if (((depart_ang < -np.pi/2) and (yaw > return_ang)) or 
                ((depart_ang > -np.pi/2) and (yaw < return_ang))):
                    angular_z = 0
                else:
                    angular_z = -(vel / arc_rad)
            prev_state = 1
    
    return angular_z, prev_state