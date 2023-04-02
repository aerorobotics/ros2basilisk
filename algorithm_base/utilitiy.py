#!/usr/bin/env python3

"""
File contains helper functions to be used throughout this package
"""
import numpy as np
from ros2basilisk.msg import SCStates

def is_key_frame(t_now, t_ref, dt, tol):
    """determine whether t_now is increment of dt w.r.t t_ref within some tolerance

    For example suppose
        t_now = 43.01, t_ref = 3.02, dt = 10.0, tol = 0.03
    then t_now is approximately 
        t_now ~ t_ref + dt*N 
    where N= 4, so returns true.
    """
    dt_near = (t_now - t_ref + 0.5*dt) % dt - 0.5*dt
    return np.abs(dt_near) < tol


def copy_SCStates(sc_states):
    
    sc_states_copy = SCStates()
    sc_states_copy.header = sc_states.header
    sc_states_copy.r_bn_n = sc_states.r_bn_n
    sc_states_copy.v_bn_n = sc_states.v_bn_n
    sc_states_copy.r_cn_n = sc_states.r_cn_n
    sc_states_copy.v_cn_n = sc_states.v_cn_n
    sc_states_copy.sigma_bn = sc_states.sigma_bn
    sc_states_copy.omega_bn_b = sc_states.omega_bn_b
    sc_states_copy.omegadot_bn_b = sc_states.omegadot_bn_b
    sc_states_copy.totalaccumdvbdy = sc_states.totalaccumdvbdy
    sc_states_copy.totalaccumdv_bn_b = sc_states.totalaccumdv_bn_b
    sc_states_copy.nonconservativeaccelpntb_b = sc_states.nonconservativeaccelpntb_b
    sc_states_copy.mrpswitchcount = sc_states.mrpswitchcount

    return sc_states_copy