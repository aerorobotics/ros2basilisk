
import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import IntEnum

from .TimeTable import TimeTable, TimeSlot

class mrp_control():
    """Attitude PD controller based on Modified Rodrigues Parameter (MRP)
    This controller will control the attitude to a fixed attitude mrp_desired
    """

    def __init__(self, Kp= 10.0, Kd= 100.0) -> None:
        self.Kp = Kp
        self.Kd = Kd

    def update(self, mrp_est, rate_est, mrp_desired):
        """Desired_att is the attitute needed for the thruster control.
        """

        rot_d = R.from_mrp(mrp_desired)
        rot_e = R.from_mrp(mrp_est)
        rot_error = rot_d.inv() * rot_e
        mrp_error = rot_error.as_mrp()
        lrCmd = mrp_error * self.Kp + np.array(rate_est) * self.Kd

        return -lrCmd

class rws_alloc():
    """Reaction wheel control allocation problem. rw_config assumes the following structure.
        
        rw_config:
            - type: "Honeywell_HR16"
            gsHat_B: [1,0,0]
            maxMomentum: 50.0
            Omega: 100.0
            - type: "Honeywell_HR16"
            gsHat_B: [0,1,0]
            maxMomentum: 50.0
            Omega: 100.0
            ....
            
    """
    
    def __init__(self, rw_config) -> None:
        
        self.config = []
        for rw in rw_config:
            rw_copy = {}
            rw_copy['type']        = rw['type']
            rw_copy['gsHat_B']     = rw['gsHat_B']
            rw_copy['maxMomentum'] = rw['maxMomentum']
            rw_copy['Omega']       = rw['Omega']
            self.config.append(rw_copy)
            
        pass
    
    def allocate(self, torque_B):
        """TODO: currently we assume we have exactly three RW's and
        each one is aligned with +x, +y, +z axes in body frame. 
        
        Negative sign because of action-reaction; torque you want to apply to RW is the 
        opposite of the torque you want to apply to the spacecraft
        """
        
        return [-torque_B[0], -torque_B[1], -torque_B[2]]
        

class AttitudeScheduler():
    """Schedule attitude guidance maneuvers.
    Given multiple attitude guidance events by different submodules, this module computes the current attitude
    based on priorities.
    """
    
    class AttMode(IntEnum):
        """Different atttiude mode. Higher the number, higher the priority."""
        DEFAULT = 0 # no attitude
        SENSOR  = 1 # point-sensor mode
        THRUST  = 2 # point-thruster mode
        
    def __init__(self) ->None:
        self.time_table = TimeTable()
        pass
    
    def add_slot(self, att_mode: AttMode, t_begin, t_end, fixed_mrp_des = None) -> None:
        att_data = {}
        att_data['att_mode'] = att_mode
        if fixed_mrp_des is not None:
            att_data['fixed_mrp_des'] = fixed_mrp_des
        time_slot = TimeSlot(t_begin, t_end, att_mode, att_data)
        self.time_table.add_slot(time_slot)
        return
    
    def get_att(self, t)->TimeSlot:
        return self.time_table.get_slot(t)
    
