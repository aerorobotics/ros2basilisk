#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

from .LinAlg import skew

class RelativeOrbitalMechanics():
        
    @classmethod
    def eci2lvlh_rate(cls,posRef_I_I,velRef_I_I): 
        """
        Purpose:
            Compute angular rate of LVLH frame from position and velocity in
            inertial frame (e.g. ECI)
        """
      
        angMom = np.cross(posRef_I_I, velRef_I_I) 
        omega_L_I = angMom/np.linalg.norm(posRef_I_I)**2
        
        return omega_L_I 


    @classmethod
    def eci2lvlh(cls,posRef_I_I,velRef_I_I):
        """
        Compute LVLH from ECI position
        """
        posRef_I_I = np.array(posRef_I_I)
        velRef_I_I = np.array(velRef_I_I)
        
        angMom = np.cross(posRef_I_I,velRef_I_I) 
        r_I = posRef_I_I/np.linalg.norm(posRef_I_I) 
        n_I = angMom/np.linalg.norm(angMom) 
        t_I = np.cross(n_I,r_I) 
        R_L_I = np.vstack((r_I,t_I,n_I))
        
        return R_L_I

    @classmethod
    def rel2abs(cls,pos_r_r,vel_r_r,rot_i_r,angRate_r_i,pos_ref_i_i,vel_ref_i_i): 
        """
        Purpose:
        This function computes the absolute position and velocity of an object
        (e.g. a spacecraft) expressed in the relative position and velocity of
        the rotating and traversing reference frame.
        
        Input:
        - pos_r_r:        relative position expressed in the rotating frame 
        - vel_r_r:        relative velocity in rotating frame, expressed in the
                        rotating frame
        - rot_i_r:        
        - angRate_r_i:
        - pos_ref_i_i:
        - vel_ref_i_i:
        
        Dependency:
        - rot2inert    
        """

        # compute the relative position and relative velocity in inertial frame
        relPos_r_i,relVel_i_i = cls.rot2inert(pos_r_r,vel_r_r,rot_i_r,angRate_r_i)
            
        # compute the absolute position and absolute velocity
        pos_i_i = pos_ref_i_i + relPos_r_i
        vel_i_i = vel_ref_i_i + relVel_i_i
        
        return pos_i_i, vel_i_i 


    @classmethod
    def rel2abs_v2(cls,pos_r_r,vel_r_r,pos_ref_i_i,vel_ref_i_i):
        """This function also computes rotation matrix and rate from reference.
        """
        R_L_I = cls.eci2lvlh(pos_ref_i_i,vel_ref_i_i)
        omega_L_I = cls.eci2lvlh_rate(pos_ref_i_i,vel_ref_i_i)
        pos_i_i, vel_i_i = cls.rel2abs(pos_r_r,vel_r_r,R_L_I.T,omega_L_I,pos_ref_i_i,vel_ref_i_i)

        return pos_i_i, vel_i_i

    @classmethod
    def inert2rot(cls,pos_r_i,vel_i_i,rot_r_i,angRate_r_i): 
        """
        Compute the position and velocity in inertial frame (i) from relative
        position and velocity in rotating frame (r).
        
        Input:
        - pos_r_i:        relative position of object with respect to the origin
                        of rotating frame, expressed in the inertial frame
        - vel_i_i:        relative velocity of object with respect to the
                        inertial frame, expressed in the inertial frame
        - rot_i_r:        rotation from inertial frame to rotating frame
        - angRage_r_i :   angular rate of the rotating frame with respect to
                        inertial frame, experssed in inertial frame
        Output:
        - pos_r_r:        relative position of object with respect to the origin 
                        of rotating frame, experessed in the rotating frame
        - vel_I_I:        relative velocity of object with respect to the
                        rotating frame, experessed in the rotating frame
        """
        pos_r_r = rot_r_i.dot(pos_r_i)
        vel_r_i = vel_i_i - np.cross(angRate_r_i,pos_r_i)
        vel_r_r = rot_r_i.dot(vel_r_i)
        
        return pos_r_r,vel_r_r

    @classmethod
    def rot2inert(cls,pos_r_r,vel_r_r,rot_i_r,angRate_r_i):
        """
        Compute the position and velocity in inertial frame (i) from relative
        position and velocity in rotating frame (r).
        
        Input:
        - pos_r_r:        relative position of object with respect to the origin 
                        of rotating frame, experessed in the rotating frame
        - vel_I_I:        relative velocity of object with respect to the
                        rotating frame, experessed in the rotating frame 
        - rot_i_r:        rotation from rotating frame to inertial frame
        - angRage_r_i :   angular rate of the rotating frame with respect to
                        inertial frame, experssed in inertial frame
        Output:
        - pos_r_i:        relative position of object with respect to the origin
                        of rotating frame, expressed in the inertial frame
        - vel_i_i:        relative velocity of object with respect to the
                        inertial frame, expressed in the inertial frame
        """

        pos_r_i = rot_i_r.dot(pos_r_r)
        vel_r_i = rot_i_r.dot(vel_r_r)
        
        vel_i_i = vel_r_i + np.cross(angRate_r_i,pos_r_i,axis = 0) 
        
        return pos_r_i,vel_i_i

    @classmethod
    def inert2rot_mat(cls,rot_r_i,angRate_r_i):
        """Equivalent to inert2rot method 
        """
        mat = np.block([[rot_r_i, np.zeros((3,3))],
                        [-rot_r_i@skew(angRate_r_i), rot_r_i]])
        return mat

    @classmethod
    def rot2inert_mat(cls,rot_i_r,angRate_r_i):
        """Equivalent to inert2rot method 
        """
        mat = np.block([[rot_i_r, np.zeros((3,3))],
                        [skew(angRate_r_i)@rot_i_r, rot_i_r]])
        return mat

    @classmethod
    def abs2rel(cls,pos_i_i, vel_i_i,rot_r_i,angRate_r_i,pos_ref_i_i,vel_ref_i_i): 
        """
        This function computes the relative position and velocity of an object
        (e.g. a spacecraft) with respect to the rotating frame. The inputs are
        the absolute positions and velocities of the object and the reference
        frame.
        """

        # relative pos/vel in inertial frame
        relPos_r_i = pos_i_i - pos_ref_i_i
        relVel_i_i = vel_i_i - vel_ref_i_i

        # compute the relative position and relative velocity in inertial frame 
        pos_r_r,vel_r_r = cls.inert2rot(relPos_r_i,relVel_i_i,rot_r_i,angRate_r_i)

        return pos_r_r,vel_r_r

    @classmethod
    def abs2rel_v2(cls,pos_i_i, vel_i_i,pos_ref_i_i,vel_ref_i_i): 
        """
        """
        rot_r_i  = cls.eci2lvlh(pos_ref_i_i,vel_ref_i_i)
        angRate_r_i = cls.eci2lvlh_rate(pos_ref_i_i,vel_ref_i_i)
        pos_r_r,vel_r_r = cls.abs2rel(pos_i_i, vel_i_i,rot_r_i,angRate_r_i,pos_ref_i_i,vel_ref_i_i)
        return pos_r_r,vel_r_r

    @classmethod
    def abs2rel_pos(cls, pos_i_i, pos_ref_i_i,vel_ref_i_i):
        """ Absolute to relative conversion, only for position"""
        rot_r_i  = cls.eci2lvlh(pos_ref_i_i,vel_ref_i_i)
        return rot_r_i @ (pos_i_i - pos_ref_i_i)

    @classmethod
    def abs2rel_batch(cls, pos_i_i_traj,vel_i_i_traj, pos_ref_i_i_traj, vel_ref_i_i_traj ):
        """ Compute absolute 2 relative in batch. Assume pos_i_i_traj is Nx3 matrix
            where N is the number of time series in trajectory. Each row must correspond to
            same time in all input variables
        """

        num = pos_i_i_traj.shape[0]
        pos_r_r_traj = []
        vel_r_r_traj = []
        for i in range(num):
            pos_r_r, vel_r_r = cls.abs2rel_v2(pos_i_i_traj[i,:], vel_i_i_traj[i,:], pos_ref_i_i_traj[i,:], vel_ref_i_i_traj[i,:])
            pos_r_r_traj.append(pos_r_r)
            vel_r_r_traj.append(vel_r_r)

        return np.array(pos_r_r_traj), np.array(vel_r_r_traj)


if __name__ == '__main__':
    n = 0.012
    dt = 10
    phi = RelativeOrbitalMechanics.computeHcwStm(n, dt)
    print(phi)