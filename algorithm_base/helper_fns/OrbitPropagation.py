#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import datetime
import numpy as np
from enum import Enum
from scipy.integrate import solve_ivp
import pyshtools as pysh # spherical harmonic tool
import pymap3d

from .PropagationUtil import PropagationUtil as propUtil
from .PropagationInterface import PropagateStates

class optionGrav(Enum):
    POINT_MASS = 1
    J2 = 2
    SH = 3

class OrbitPropagation(PropagateStates):
    """Numerical propagation of translational motion of spacecraft in planetary orbits
    """
    
    def __init__(self, gravConst, gravity_option = "point mass", enable_drag = False, **kwargs):

        self.gravConst = gravConst
        self.idx_pos = [0,1,2] # must be consecutive
        self.idx_vel = [3,4,5] # must be consecutive
        self.dim_state = len(self.idx_pos) + len(self.idx_vel)


        # gravity options
        if gravity_option == "point mass":
            self.gravity_option = optionGrav.POINT_MASS
        elif gravity_option == "J2":
            self.gravity_option = optionGrav.J2
            self.planetRadius = kwargs["planetRadius"]
            self.j2 = kwargs["J2"]        
        elif gravity_option == "shg":
            self.gravity_option = optionGrav.SH
            print("Loading SHG Coefficients...")
            self.clm = pysh.datasets.Earth.EGM2008() # load gravity coefficients
            print("--> Complete.")
            self.date_ref = kwargs["date_time_ref"]
        else: 
            raise ValueError("select a valid gravity option")

        
        if "process_noise_cov" in kwargs:
            self.proc_mat = kwargs["process_noise_cov"]
        
        if enable_drag:
            self.sat_mass = kwargs["sat_mass"]
            self.sat_cd   = kwargs["sat_drag_coeff"]
            self.sat_area = kwargs["sat_area"]
                
    
    def eomOrbit(self,t,x): 
        """
        Purpose: 
            comptue the equation of motion 
        Input:
        - t:      NOT USED. place holder needed for odeint
        - x:      state of object in planetary orbit [6x1]  where
                      x' = [pos_I_I' vel_I_I']
                  where
                      pos_I_I: position from ECI expressed in ECI (m) [3x1]
                      vel_I_I: velocity from ECI expressed in ECI (m/s) [3x1]
        """
        
        # point mass gravity acceleration
        pos_I_I = x[self.idx_pos]
                
        if self.gravity_option == optionGrav.POINT_MASS:
            accGrav = self.gravPointMass(pos_I_I)
        elif self.gravity_option == optionGrav.J2:
            accGrav = self.gravJ2(pos_I_I, self.planetRadius, self.gravConst, self.j2)
        elif self.gravity_option == optionGrav.SH:
            accGrav = self.gravShg(pos_I_I, t)
            
        # TODO: implement these
        accDrag = np.zeros(3)   # aerodynamic drag (high priority)
        
        # add all forces 
        velDot = accGrav + accDrag
        xdot = np.concatenate([x[self.idx_vel],velDot]) 
        
        return xdot 

    
    def eom_state_cov(self,t,state_and_cov): 
        """ Equation of motion for both states and covariance
        """
        state = state_and_cov[0:self.dim_state]
        cov_vec = state_and_cov[self.dim_state:]         
        state_dot = self.eomOrbit(t,state) # state time-derivative

        # covariance matrix
        cov_mat = cov_vec.reshape((self.dim_state,self.dim_state))

        # jacobian w.r.t states
        jac = self.jac(state)
        cov_dot = jac @ cov_mat + cov_mat @ jac.T + self.proc_mat
        
        return np.concatenate((state_dot,np.ravel(cov_dot)))
    
    
    def gravPointMass(self, posI):
        
        posI = np.array(posI)
        return -self.gravConst/(np.linalg.norm(posI[0:3])**3)*posI[0:3]
    
    
    def jac(self, state):
        """ This function returns F = df/dx where the continous-time dynamics
        is defined as
            xdot = f(x,t)
        """

        jac = np.zeros((self.dim_state,self.dim_state))
        jac[self.idx_pos[0]:(self.idx_pos[-1]+1),self.idx_vel[0]:(self.idx_vel[-1]+1)] = np.eye(len(self.idx_pos))

        if self.gravity_option == optionGrav.POINT_MASS:
            jac_a_p_grav = self.jac_gravPointMass(state[self.idx_pos])
        elif self.gravity_option == optionGrav.J2:
            jac_a_p_grav = self.jac_J2(state[self.idx_pos],self.planetRadius,self.gravConst,self.j2)
        elif self.gravity_option == optionGrav.SH:
            raise NotImplementedError("Jacobian for Spherical harmonics is not implemented yet")
            
        jac[self.idx_vel[0]:(self.idx_vel[-1]+1),self.idx_pos[0]:(self.idx_pos[-1]+1)] = jac_a_p_grav

        return jac
        
    
    def jac_gravPointMass(self, posI):
        """
        Purpose:
            compute jacobian of point-mass gravitational acceleration with respect to position
        """
        posI = np.array(posI)
        posI_norm = np.linalg.norm(posI[0:3])
        jac = -self.gravConst/(posI_norm**3)*np.eye(3) + 3*self.gravConst/(posI_norm**5)* np.outer(posI,posI)
        return jac


    
    def gravJ2(self, posI, planetRadius, gravConst, j2):
        """
        Purpose: 
            Compute the acceleration due to the j2 perterbations
        Input:
        - posI:         inertial position of object (e.g. ECI position) (m) (3,)
        - planetRadius: radius of the planet 
        - gravConst:    graviational constant of the planet (m^3/s^2)
        - J2:           J2 coefficient of the planet

        Output:
        - accI:         acceleration due to J2 perturbation in inertial frame (m/s^2) (3,)
        """

        ri = posI[0]
        rj = posI[1]
        rk = posI[2]
        r = np.linalg.norm(posI[0:3])

        rkNormSqrr = (rk/r)**2
        coeff = -3*j2*gravConst*planetRadius**2/(2*r**5)
        xacc = coeff*ri*(1-5*rkNormSqrr)
        yacc = coeff*rj*(1-5*rkNormSqrr)
        zacc = coeff*rk*(3-5*rkNormSqrr)
        
        gravPointMass = self.gravPointMass(posI)
        
        return gravPointMass + np.array([xacc, yacc, zacc])
        # return np.array([xacc, yacc, zacc])

    def jac_J2(self, posI, planetRadius, gravConst, j2):
        # TODO: VERIFY this function is implemented correctly.
        ri = posI[0]
        rj = posI[1]
        rk = posI[2]
        r = np.linalg.norm(posI[0:3])

        rkNormSqrr = (rk/r)**2
        coeff = -3*j2*gravConst*planetRadius**2/(2*r**5)
        vec = np.array([ri*(1-5*rkNormSqrr),
                        rj*(1-5*rkNormSqrr),
                        rk*(3-5*rkNormSqrr)])
        
        rkNormSqrr_grad = -2*rk**2/r**4*posI + 2*rk/r**2*np.array([0,0,1])
        coeff_grad = 15*j2*gravConst*planetRadius**2/(2*r**7)*posI
        vec_grad = np.diag([(1-5*rkNormSqrr), (1-5*rkNormSqrr), (3-5*rkNormSqrr)]) \
                  -5*np.outer(posI, rkNormSqrr_grad)

        jac_j2 = np.outer(vec,coeff_grad) + coeff*vec_grad
        jac_pt = self.jac_gravPointMass(posI)

        return jac_pt + jac_j2
        # return jac_j2


    def gravShg(self, posI, dt):
        """Spherical Harmonic Gravity
        This function has been tested against Matlab gravityspherical harmonic.
        """

        date_now = self.date_ref + datetime.timedelta(seconds=dt)
        deg_max = 120

        # first convert ECI --> ECEF
        pos_ecef = pymap3d.eci2ecef(posI[0],posI[1], posI[2], date_now)

        # then convert ECEF --> circular radius, latitude, longitude (not geodetic)
        rlatlon = _xyz2rlatlon(pos_ecef)

        # compute gravity in radius, lat, lon frame.
        g_rlatlon = pysh.gravmag.MakeGravGridPoint(\
                        self.clm.coeffs, 
                        self.clm.gm,
                        self.clm.r0,
                        rlatlon[0], 
                        rlatlon[1]*180/np.pi, # lat lon must be in degrees
                        rlatlon[2]*180/np.pi,
                        lmax = deg_max)
        
        # rotate RLL frame to ECEF
        R_rll2ecef = _mat_rlatlon2xzy(rlatlon[1],rlatlon[2])
        g_ecef = R_rll2ecef @ g_rlatlon

        # finally rotate it back to ECEF
        return np.array(pymap3d.ecef2eci(g_ecef[0],g_ecef[1], g_ecef[2], date_now))
    
    # override abstract method
    def propagate_state(self, ti, tf, state_init, control = None):
        """Implementation of base class"""
        return self.propagateOnce(tf-ti, state_init, control)
    
    def propagate(self, t_vec, state, covariance = None):
        """
        Note:
        t_vec[0] is the first point
        """
        num_time = len(t_vec)
        
        # covariance
        if covariance is not None:
            var = np.concatenate((state, np.ravel(covariance)))
            state_series = [state]
            cov_series = [covariance]
        else:
            var = state
            state_series = [var] 

        for i in range(num_time-1):
            tbegin = t_vec[i]
            tend   = t_vec[i+1]
            if covariance is not None:
                sol = solve_ivp(self.eom_state_cov,[tbegin, tend],var)
            else:
                sol = solve_ivp(self.eomOrbit,[tbegin, tend],var)
            
            var = sol.y[:,-1] 

            if covariance is not None:
                state_series.append(var[0:self.dim_state])
                cov_series.append(np.reshape(var[self.dim_state:],(self.dim_state,self.dim_state)))
            else:
                state_series.append(var)

        if covariance is not None:
            return state_series, cov_series
        else:
            return state_series

    
    def propagateOnce(self,dt, posvel_ECI, control = None):
        # solutionOrbit = solve_ivp(self.eomOrbit,[0, dt],posvel_ECI)
        # return solutionOrbit.y[:,-1]
        return propUtil.propagateOnce(self.eomOrbit,dt, posvel_ECI, control)

    
    
    def save_trajectory(self, state_init, t_vec, sc_name, file_path):
        """Computes satellite trajectories and save in JSON format.
        """
        
        
        # generate trajectory
        state_series = self.propagate(t_vec, state_init)
        
        temp = 0
        # filePath = file_path + sc_name +'.csv'            
        # with open(filePath, 'w', newline='') as csvfile:
        #     write = csv.writer(csvfile, delimiter=',')
        #     for i in range(len(out['stateTrue'][scNameChief]['stateAbs'])):
        #         write.writerow(out['stateTrue'][scName]['stateAbs'][i,:].tolist())


def _xyz2rlatlon(pos):
    """ecef to radius, lat, lon"""
    r = np.linalg.norm(pos[0:3])
    lat = np.arctan2(pos[2],np.sqrt(pos[0]**2+pos[1]**2))
    lon = np.arctan2(pos[1],pos[0])
    return np.array([r,lat,lon])

def _mat_rlatlon2xzy(lat,lon):
    """Get (pseudo) Rotation matrix from (r,lat,lon) to (x,y,z)

    It's pseudo because r lat lon is left handed (det(mat)=-1). 
    not that it matters.
    """
    sla = np.sin(lat)
    cla = np.cos(lat)
    slo = np.sin(lon)
    clo = np.cos(lon)
    return np.array([[cla*clo, -sla*clo, -slo],
                     [cla*slo, -sla*slo,  clo],
                     [sla,      cla,      0.0]])

if __name__ == "__main__":
    
    import pyproj

    # sample values
    EARTH_GRAV_CONST = 3.986004418E14 # Standard gravitaitonal parameter of Earth [m^3/s^2] 
    state = np.array([6400000.0,0,0, 0,7000,0])
    cov = np.eye(len(state))
    dt = 0.0
    
    r_ecef = np.array([1501277.4766305785, 6221417.070058232, 11643.820238739392])

    ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
    lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
    lon, lat, alt = pyproj.transform(ecef, lla, r_ecef[0], r_ecef[1], r_ecef[2], radians=True)

    # 
    
    utc = datetime.datetime(2019, 1, 4, 12)
    prop = OrbitPropagation(gravConst=EARTH_GRAV_CONST, gravity_option="shg", date_time_ref = utc)
    g = prop.gravShg(state[0:3],dt)
    # state_series, cov_series = prop.propagate([0,dt],state)

    print(g)
    pass