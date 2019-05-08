# -*- coding: utf-8 -*-
"""
Helper function to perform scaling transform of dimensions from BU
Shared Robotics Lab model space to Mcity real-world CAV environment.
Scale is set to the ratio of the max velocity of the robots to the 
max velocity of the M-city CAVs.
    
Questions? sherwinl@bu.edu, BU CODES Lab
"""
scale = 13/0.4

def lab_to_mcity(dim):
    """
    Converts model dimension to real world.
    """
    return dim * scale
    
def mcity_to_lab(dim):
    """
    Converts real-world dimension to model.
    """
    return dim / scale