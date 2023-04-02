#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from scipy.spatial.transform import Rotation as R

def look_at_mat(pos, eye, up):
    # compute body attitude orientation in a reference frame such that camera located at 
    # "eye" will look at object "pos"
    # Inputs:
    #  - eye: position of camera in reference frame
    #  - pos: position of target object in reference frame
    #  - up:  up direction to fully constrain the problem. 
    #         cannot be colinear with (pos - eye) vector
    # Output:
    #  - body_rot: camera body orientation such that camera points to the object
    #
    # Assume: +Z axis of the body frame is the line-of-sight.
    ################################################################
    pos = pos.reshape(3, )
    eye = eye.reshape(3, )
    up = up.reshape(3, )

    z = pos - eye
    z = z/np.linalg.norm(z)

    y = up/np.linalg.norm(up)

    x = np.cross(y, z)
    x = x/np.linalg.norm(x)

    y = np.cross(z, x)
    y = y/np.linalg.norm(x)

    return R.from_matrix(np.concatenate([x.reshape(3,1), y.reshape(3,1), z.reshape(3,1)], axis=1))


def point_sensor_mat(dir1_B, dir1_R, dir2_B, dir2_R):
    """Returns rotation matrix for an orientation such that
        - dir1_B (body frame) exactly points to dir1_R (reference frame)
        - dir2_B (body frame) tries best to point to dir2_R (reference frame)
    """
    
    x_unit_B = dir1_B/np.linalg.norm(dir1_B)
    x_unit_R = dir1_R/np.linalg.norm(dir1_R)
    
    z_unit_B = np.cross(dir1_B, dir2_B)
    z_unit_B = z_unit_B/np.linalg.norm(z_unit_B)
    z_unit_R = np.cross(dir1_R, dir2_R)
    z_unit_R = z_unit_R/np.linalg.norm(z_unit_R)
    
    y_unit_B = np.cross(z_unit_B, x_unit_B)
    y_unit_R = np.cross(z_unit_R, x_unit_R)
    
    rotmat = np.outer(x_unit_R, x_unit_B) + np.outer(y_unit_R, y_unit_B) + np.outer(z_unit_R, z_unit_B)
    
    return R.from_matrix(rotmat)
    

def skew(w): 
    w_cross = np.array([[    0, -w[2],  w[1]],
                        [ w[2],     0, -w[0]],
                        [-w[1],  w[0],     0]]) 
    return w_cross 


if __name__ == "__main__":

    
    dir1_B = np.random.randn(3)
    dir2_B = np.random.randn(3)
    dir1_R = np.random.randn(3)
    dir2_R = np.random.randn(3)
    
    # ensure unit norm
    dir1_B = dir1_B/np.linalg.norm(dir1_B)
    dir2_B = dir2_B/np.linalg.norm(dir2_B)
    dir1_R = dir1_R/np.linalg.norm(dir1_R)
    dir2_R = dir2_R/np.linalg.norm(dir2_R)
    
    rot = point_sensor_mat(dir1_B, dir1_R, dir2_B, dir2_R)
    mat_R_B = rot.as_matrix()
    
    print(np.linalg.det(mat_R_B))
    print( mat_R_B @ mat_R_B.T )
    print( mat_R_B.T @ mat_R_B )
    
    print(dir1_R)
    print(mat_R_B @ dir1_B)