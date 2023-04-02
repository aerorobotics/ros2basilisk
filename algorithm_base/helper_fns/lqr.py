"""
LQR controller
"""
import control
import numpy as np
import scipy

from helper_fns.RelativeOrbitalMechanics import RelativeOrbitalMechanics as rom
from helper_fns.OrbitalMechanics import OrbitalMechanics as om
from helper_fns.Hcw import Hcw
from helper_fns.OrbitPropagation import OrbitPropagation
from helper_fns.AttitudeUtil import look_at_mat, point_sensor_mat


class Lqr:
    """This class implements LQR-based controller for formation flying spacecraft
    """

    def __init__(self, semi_major, dt_lqr, grav_const) -> None:
        n = om.mean_motion_ideal(semi_major, grav_const)

        # hcw state transition matrix
        A = Hcw.compute_stm(n, dt_lqr)
        B = A[:, 3:6]  # assume impulsive delta-v vector is applied
        sigma_lqr_x_pos = 1.0
        sigma_lqr_x_vel = 0.1
        sigma_lqr_u = 10.0
        Q = scipy.linalg.block_diag(sigma_lqr_x_pos * np.eye(3), sigma_lqr_x_vel * np.eye(3))
        R = sigma_lqr_u * np.eye(3)
        K_lqr, _, _ = control.dlqr(A, B, Q, R)
        self.dt_lqr = dt_lqr
        self.A = A
        self.K_lqr = K_lqr
        self.orbit_propagator = OrbitPropagation(grav_const)

        pass

    def update(self, posvel_self_eci, posvel_ref_eci, dt_lag):
        """
        Compute the delta-V command for the next control step
        
        This function is meant to be called only every dt_ctrl sec.
        """


        # exact propagation to next LQR time step
        posvel_ref_next  = self.orbit_propagator.propagateOnce(dt_lag, posvel_ref_eci)
        posvel_self_next = self.orbit_propagator.propagateOnce(dt_lag, posvel_self_eci)

        # position and velocity in LVLH frame in next step
        pos_L_next, vel_L_next = rom.abs2rel_v2(
            posvel_self_next[0:3],
            posvel_self_next[3:6],
            posvel_ref_next[0:3],
            posvel_ref_next[3:6],
            )

        posvel_L_next = np.concatenate((pos_L_next, vel_L_next))
        posvel_l_d = np.array([0.0, 30.0, 0.0, 0.0, 0.0, 0.0])

        # control is in terms of applied delta-V (m/s)
        # desired velocity/needed velocity (for thrusters).
        # u = -K_lqr @ (posvel_l_est - posvel_l_d)
        dv_L_next = -self.K_lqr @ (posvel_L_next - posvel_l_d)

        R_L_I_next = rom.eci2lvlh(posvel_ref_next[0:3], posvel_ref_next[3:6])
        dv_I_next = R_L_I_next.T @ dv_L_next

        return dv_I_next


def get_attitude_thrust(dv_desired_I, thruster_vec_B):
    """Given some thrust direction u_I, compute attitude such that thruster
       is oriented in the direction.
       
       Assume single thruster oriented in direction of thruster_vec_B

    Args:
        dv_desired_I:   desired dV vector in inertial frame (magnitude does not matter)
        thruster_vec_B: thrust vector in body frame

    Returns:
        mrp_I_B: spacecraft attitude in Modified Rodrigues Parmaeters (MRP)
    """
    
    # avoid divide by zero when delta-v = zero
    if np.linalg.norm(dv_desired_I) < 1.0E-10:
        return np.zeros(3)
    
    
    # todo: we would like to minimize the attitude change...
    # should be able to do this by smartly selecting vec_perp... but does this make sense?
    vec_perp = np.cross(thruster_vec_B, dv_desired_I)
    
    rot_I_B = point_sensor_mat(thruster_vec_B, dv_desired_I, vec_perp, vec_perp)
    
    return rot_I_B.as_mrp()
