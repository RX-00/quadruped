#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 18:16:57 2020

@author: linux-asd
"""

import numpy as np
import math 

def Rx(roll):
    """ Rotation matrix arround x (roll)
    """
    return np.matrix([[1,            0,             0, 0],
                      [0, np.cos(roll), -np.sin(roll), 0],
                      [0, np.sin(roll),  np.cos(roll), 0],
                      [0,            0,             0, 1]])

def Ry(pitch):
    """ Rotation matrix arround y (pitch)
    """
    return np.matrix([[ np.cos(pitch), 0, np.sin(pitch), 0],
                      [             0, 1,             0, 0],
                      [-np.sin(pitch), 0, np.cos(pitch), 0],
                      [             0, 0,             0, 1]])

def Rz(yaw):
    """ Rotation matrix arround z (yaw)
    """
    return np.matrix([[np.cos(yaw), -np.sin(yaw), 0, 0],
                      [np.sin(yaw),  np.cos(yaw), 0, 0],
                      [          0,            0, 1, 0],
                      [          0,            0, 0, 1]])
    
def Rxyz(roll , pitch , yaw):
    if roll != 0 or pitch != 0 or yaw != 0:
        R = Rx(roll)*Ry(pitch)*Rz(yaw)
        return R
    else:
        return np.identity(4)
    

def RTmatrix(orientation, position):
    "translation and rotation compose"
    roll = orientation[0]
    pitch = orientation[1]
    yaw = orientation[2]
    x0 = position[0]
    y0 = position[1]
    z0 = position[2]
    
    translation = np.matrix([[1, 0, 0, x0],#Translation matrix
                             [0, 1, 0, y0],
                             [0, 0, 1, z0],
                             [0, 0, 0,  1]])
    rotation = Rxyz(roll, pitch, yaw)#rotation matrix
    
    return rotation*translation
    
def transform(coord,rotation,translation):
    "transforms a vector to a desire rotation and translation"
    vector = np.matrix([[coord[0,0]],
                        [coord[0,1]],
                        [coord[0,2]],
                        [         1]])
    
    tranformVector = RTmatrix(rotation,translation)*vector
    #the last element of the vector is depreciated
    return np.matrix([tranformVector[0,0], tranformVector[1,0], tranformVector[2,0]])
#w 0  x 1 y 2 z 3
def q_to_eular(q):
    rol =  math.atan2(2*(q[2]*q[3]+q[0]*q[1]),q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])
    pit =  math.asin(2*(q[1]*q[3]-q[0]*q[2]))
    yaw =  math.atan2(2*(q[1]*q[2]+q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3])
    return [ rol, pit , yaw]

