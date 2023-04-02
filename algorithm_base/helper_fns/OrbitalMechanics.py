#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import kepler # import from https://pypi.org/project/kepler.py/
from numpy import linalg as la

from . import Constants as const

class OrbitalMechanics():
    """Utility functions commonly used in Orbital Mechanics
    """
    MAX_ITERATIONS = 100

    @classmethod
    def oe_vector(cls,a,e,i,raan,w,true_anom=None, mean_anom=None):
        if (true_anom is not None):
            return [a, e, i, raan, w, true_anom]
        
        if (mean_anom is not None):
            tanom = cls.true_anomaly_from_mean(e,mean_anom)
            return [a, e, i, raan, w, tanom]
        
        # incorrect input    
        raise ValueError("Incorrect input type")

    @classmethod
    def oe2eci(cls, mu_p, oev):
        """Converts the orbital elements to ECI position and velocity."""
                
        sma    = oev[0]
        ecc    = oev[1]
        inc    = oev[2]
        argper = oev[3]
        raan   = oev[4]
        tanom  = oev[5]

        slr = sma * (1 - ecc * ecc)
        rm = slr / (1 + ecc * np.cos(tanom))

        arglat = argper + tanom

        sarglat = np.sin(arglat)
        carglat = np.cos(arglat)

        c4 = np.sqrt(mu_p / slr)
        c5 = ecc * np.cos(argper) + carglat
        c6 = ecc * np.sin(argper) + sarglat

        sinc = np.sin(inc)
        cinc = np.cos(inc)

        sraan = np.sin(raan)
        craan = np.cos(raan)

        # position vector
        r_ECI = np.array([rm * (craan * carglat - sraan * cinc * sarglat),
                rm * (sraan * carglat + cinc * sarglat * craan),
                rm * sinc * sarglat])

        # velocity vector
        v_ECI = np.array([-c4 * (craan * c6 + sraan * cinc * c5),
                -c4 * (sraan * c6 - craan * cinc * c5),
                c4 * c5 * sinc])

        return r_ECI,v_ECI  

    @classmethod
    def eci2oe(cls,mu_p, r_ECI,v_ECI):
        """Converts ECI position velocity to orbital elements.

        References:
        [1] Vallado,D. Fundamentals of Astrodynamics and Applications. 2007.
        """

        mu = mu_p

        pos = r_ECI
        vel = v_ECI

        h  = np.cross(pos,vel)          # angular momentum
        ni = np.array([-h[1], h[0], 0]) # normal vector

        rMag = la.norm(pos)
        vMag = la.norm(vel)
        hMag = la.norm(h)
        nMag = la.norm(ni)

        evec = ((vMag**2-mu/rMag)*pos-np.dot(pos,vel)*vel)/mu # eccentricity vector
        e = la.norm(evec) # eccentricity


        tol = 1E-8
        xi = vMag**2/2-mu/rMag 

        if np.abs(e-1.0) < tol: # parabolic
            # p = h**2/mu
            a = np.inf
        else:
            a = -mu/(2*xi)
            # p = a*(1-e)
        
        # inclination
        i = np.arccos(h[2]/hMag)       

        # RAAN
        Omega = np.arccos(ni[0]/nMag) 
        if ni[1] < 0:
            Omega = 2*np.pi-Omega

        # Argument of perigee
        w = np.arccos(np.dot(ni,evec)/(nMag*e))
        if evec[2] < 0:
            w = 2*np.pi-w

        # True Anomaly
        nu = np.arccos(np.dot(evec,pos)/(e*rMag))
        if np.dot(pos,vel) < 0:
            nu = 2*np.pi-nu

        oev = np.array([a,e,i,w,Omega,nu])

        return oev

    @classmethod
    def eccentric_anomaly_from_mean(cls,e, M, tolerance=1e-14):
        E = kepler.solve(M, e)
        return E

    @classmethod
    def mean_anomaly_from_eccentric(cls, e, E):
        return E-e*np.sin(E)

    # @classmethod
    # def eccentric_anomaly_from_mean(cls,e, M, tolerance=1e-14):
    #     """Convert mean anomaly to eccentric anomaly.
    #     Implemented from [A Practical Method for Solving the Kepler Equation][1]
    #     by Marc A. Murison from the U.S. Naval Observatory
    #     [1]: http://murison.alpheratz.net/dynamics/twobody/KeplerIterations_summary.pdf
    #
    #     Kai: unfortunately, this was not very stable in some e, M. 
    #     """
    #     Mnorm = np.fmod(M, 2 * np.pi)
    #     E0 = M + (-1 / 2 * e ** 3 + e + (e ** 2 + 3 / 2 * np.cos(M) * e ** 3) * np.cos(M)) * np.sin(M)
    #     dE = tolerance + 1
    #     count = 0
    #     print("tolerance", tolerance)
    #     while dE > tolerance:
    #         t1 = np.cos(E0)
    #         t2 = -1 + e * t1
    #         t3 = np.sin(E0)
    #         t4 = e * t3
    #         t5 = -E0 + t4 + Mnorm
    #         t6 = t5 / (1 / 2 * t5 * t4 / t2 + t2)
    #         E = E0 - t5 / ((1 / 2 * t3 - 1 / 6 * t1 * t6) * e * t6 + t2)
    #         dE = np.abs(E - E0)
    #         print("t2=", t2, ", t3=", t3, ", t4=", t4, ", t5=", t5, ", t6=", t6, ", E=",E, ", dE=",dE)
    #         E0 = E
    #         count += 1
    #         if count == cls.MAX_ITERATIONS:
    #             raise ValueError('Did not converge after {n} iterations. (e={e!r}, M={M!r})'.format(n=cls.MAX_ITERATIONS, e=e, M=M))
    #     return E

    @classmethod
    def true_anomaly_from_eccentric(cls, e, E):
        """Convert eccentric anomaly to true anomaly."""
        return 2 * np.arctan2(np.sqrt(1 + e) * np.sin(E / 2), np.sqrt(1 - e) * np.cos(E / 2))

    @classmethod
    def true_anomaly_from_mean(cls, e, M, tolerance=1e-14):
        """Convert mean anomaly to true anomaly."""
        E = cls.eccentric_anomaly_from_mean(e, M, tolerance)
        return cls.true_anomaly_from_eccentric(e, E)

    @classmethod
    def mean_motion_ideal(cls, sma, mu_p = const.EARTH_GRAV_CONST):
        return np.sqrt(mu_p/sma**3)

    @classmethod
    def quick_init(cls, sma = 6378136.6+200, ecc = 0.0, inc = 0.0, argper = 0.0, raan = 0.0, tanom = 0.0):
        """Initialize the ECI position from orbital elements"""
        oev = np.array([sma, ecc, inc, argper, raan, tanom])
        pos_eci = cls.oe2eci(const.EARTH_GRAV_CONST, oev)
        return pos_eci
