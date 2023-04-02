
import numpy as np


class Hcw():
    """Library of tools related to Hill-Clohessy-Wiltshire equations (HCW)
    This assumes
        - Orbital dynamics is given by Kapler's motion
        - eccentricity is small (circular orbitt)
        - separation distance is small
    """
    
    @classmethod
    def compute_stm(cls,n,dt):
        """
        Compute the state transition matrix (STM) based on the HCW equations
        """

        nt = n*dt
        cnt = np.cos(nt)
        snt = np.sin(nt)

        Phi_rr = np.array([[4-3*cnt, 0, 0],
                            [6*(snt-nt),  1,  0],
                            [0, 0, cnt]])
        
        Phi_rv = np.array([[snt/n,  (1-cnt)*2/n, 0],
                           [(cnt-1)*2/n,  (4*snt-3*nt)/n,  0],
                           [0, 0, snt/n]])

        Phi_vr = np.array([[3*n*snt,0,0],
                           [6*n*(cnt-1),0,0],
                           [0,0,-n*snt]])

        Phi_vv = np.array([[cnt, 2*snt, 0],
                           [-2*snt, 4*cnt-3, 0],
                           [0, 0, cnt]])

        return np.block([[Phi_rr,  Phi_rv], [Phi_vr,  Phi_vv]])
    
    @classmethod
    def actuation_mat_impulsive(cls,n,dt):
        """
        Compute the control actuation matrix for 3-axis impulsive control.
        This is essentially the second block column of the HCW matrix
        """

        nt = n*dt
        cnt = np.cos(nt)
        snt = np.sin(nt)

        Phi_rv = np.array([[snt/n,  (1-cnt)*2/n, 0],
                           [(cnt-1)*2/n,  (4*snt-3*nt)/n,  0],
                           [0, 0, snt/n]])
        Phi_vv = np.array([[cnt, 2*snt, 0],
                           [-2*snt, 4*cnt-3, 0],
                           [0, 0, cnt]])
                           
        return np.block([[Phi_rv], [Phi_vv]]) # block column concatenation


    @classmethod
    def pro_constraint(cls,n):
        """
        This function returns matrix H such that
            H x = 0
        satisfies the PRO constraint (no-drift constraint) which is given by
            v_T + 2* n* r_R = 0
        """
        return np.array([2*n,0,0,0,1,0])



    @classmethod
    def get_pro(cls, n, A0 = 0.0,B0 = 0.0, alpha = 0.0, beta = 0.0, yoff = 0.0):
        """
        This function finds the initial relative position and velocity of that 
        satisfies the no-drift constraint (aka, the resulting trajectory is PRO).
        The initial condition is specified as the amplitude and phase of the
        relative trajectory.

        These parameters completely describes all the possible no-drift PRO's with HCW.

        A0 specifies the in-track ellipsoid diameter
        B0 specifies the cross-track motion amplitude
        alpha specifies the phase of the in-track motion
        beta specifies the phase of the cross-track motion
        yoff specifies the along-track separation

        Author:
        Kai Matsuka
        
        Ref:
        - /hcw/HCW_writeup.pdf, see PRO page
        """
        x0 = np.array([A0*np.cos(alpha),
                       -2*A0*np.sin(alpha)+yoff,
                        B0*np.cos(beta),
                       -n*A0*np.sin(alpha),
                       -2*n*A0*np.cos(alpha),
                       -n*B0*np.sin(beta)])

        return x0 


    @classmethod
    def alongTrackFormation(cls,meanAnom,yoff_list):
        """
        This function returns relative position and velocities that yields
        along track formation
        """
        N = len(yoff_list)
        posvel = []

        A0 = 0 
        B0 = 0
        alpha = 0 
        beta = 0

        #for ii in np.nditer(x0,order='F'):
        for ii in range(N):
            posvel.append(cls.get_pro(meanAnom,A0,B0,alpha,beta,yoff_list[ii]))
        
        return posvel

    @classmethod
    def crossTrackFormation(cls,meanAnom,B0,yoff_list,flagStationaryRef = True):
        """
        This function returns relative position and velocities that yields
        cross track formation
        """
        N = len(yoff_list)
        posvel = []

        A0 = 0.0
        alpha = 0.0 
        
        if flagStationaryRef:
            posvel.append(cls.get_pro(meanAnom,A0,0.0,alpha,0.0,yoff_list[0]))
            beta_list = np.linspace(0.0, 2*np.pi, num=(N-1), endpoint=False)
            for ii in range(N-1):
                posvel.append(cls.get_pro(meanAnom,A0,B0,alpha,beta_list[ii],yoff_list[ii+1]))
        else:
            beta_list = np.linspace(0.0, 2*np.pi, num=N, endpoint=False)
            for ii in range(N):
                posvel.append(cls.get_pro(meanAnom,A0,B0,alpha,beta_list[ii],yoff_list[ii]))
            
        return posvel
    
