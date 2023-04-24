#!/usr/bin/env python3

import numpy as np

# ros imports
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber


# ros2 interfaces
from ros2basilisk.msg import SCStates, ThrustCmd, AttGuid

# custom utility
from utility import is_key_frame

from scipy.spatial.transform import Rotation as R
from helper_fns.lqr import Lqr, get_attitude_thrust
from helper_fns import Constants as const
from helper_fns.utility import yamlLoad
from helper_fns.attitude_ctrl import AttitudeScheduler
# from swarm_ff_py.SpacecraftData import SpacecraftData

class FormationControl(Node):
    """Implements 6DOF guidance & control of a chaser spacecraft for formation keeping task.
    
    At each time step of LQR, spacecraft applies an impulsive maneuver. Spacecraft pre-computes the necessary delta-v in next step. 

    Timing Schedule:
        ||                                                      ||
        ||<--------------------LQR time step------------------->||
        ||                                                      ||
        ||       |                                      |       ||
        ||<--1-->|<----------------3------------------->|<--2-->||
        ||       |                                      |       ||
    where
        1: thruster burn_duration
        2: minimum attitude settle time (time_attitude_settle)
        3: (optional) other rotational maneuvers 
        
    ASSUME:
    - spacecraft has exactly one "impulsive" thruster that is aligned to COM
    - header time stamps on message is synchronized to be an integer multiple of dt_lqr w.r.t. self.time_ref
    """

    def __init__(self) -> None:
        super().__init__("FormationControl")
        
        ########################################################
        # ROS parameters
        ########################################################
        
        # load config variables
        self.declare_parameter("workspace_dir", "/home/ubuntu/example_ws/")
        self.declare_parameter("config_yaml", "src/ros2basilisk/config/lqr_config.yaml")
        self.declare_parameter("enable_att_passthru", False)

        ws = self.get_parameter('workspace_dir').get_parameter_value().string_value
        yaml_path = self.get_parameter('config_yaml').get_parameter_value().string_value
        
        # enable_att_passthru is a boolean flag that determines attitude guidance behavior
        # in between the pointing manuevers necessary for thrust
        self.enable_att_passthru = self.get_parameter('enable_att_passthru').value
        
        ########################################################
        # Load YAML config parameters (LQR)
        ########################################################
        
        config = yamlLoad(ws + yaml_path)
        lqr_config = config["lqr_config"]

        self.dt_lqr   = lqr_config["dtLqr"]    # sec LQR control interval
        self.time_ref = lqr_config["timeRef"]  # seconds
        self.tol      = lqr_config["timeTol"]  # tolerance
        
        # ASSUME:
        # - there exists exactly single thruster
        # - thruster is aligned with COM
        self.thruster_vec = np.array(lqr_config['thrusters'][0]['tHat_B'])
        self.mass   = lqr_config["scMass"]
        self.thrust = lqr_config["thrMaxThrust"]
        self.time_attitude_settle = lqr_config["timeAttConv"]
        semi_major = lqr_config["semiMajoInit"]
        
        # sanity check
        assert(self.dt_lqr > self.time_attitude_settle)
        
        ########################################################
        # Load algorithms (LQR & AttitudeScheduler)
        ########################################################
        
        self.lqr = Lqr(semi_major, self.dt_lqr, const.EARTH_GRAV_CONST)
        self.att_scheduler = AttitudeScheduler()
        
        ########################################################
        # ROS Publisher / Subscriber
        ########################################################
        
        # create synchronized subscriber
        state_sub_self = Subscriber(self,SCStates, "/sc_self/state_nav")
        state_sub_ref  = Subscriber(self,SCStates, "/sc_ref/state_nav")
        
        if self.enable_att_passthru:
            default_att_sub = Subscriber(self, AttGuid, "/sc_self/mrp_des_sensor")
            self.sync = TimeSynchronizer( [state_sub_self, state_sub_ref, default_att_sub], 10 )
            self.sync.registerCallback(self.control_callback_passthru)
            # by defaut, pass thru pointing request by sensor
            self.att_scheduler.add_slot(AttitudeScheduler.AttMode.SENSOR, 0, float('inf') )
        else:
            self.sync = TimeSynchronizer( [state_sub_self, state_sub_ref], 10 )
            self.sync.registerCallback(self.control_callback)
            # by default, schedule is empty
            self.att_scheduler.add_slot(AttitudeScheduler.AttMode.DEFAULT, 0, float('inf') )
            
        # create publishers
        self.thrust_publisher = self.create_publisher(ThrustCmd, "/sc_self/thr_cmd", 5)
        self.att_cmd_publisher = self.create_publisher(AttGuid, "/sc_self/mrp_des", 5)

        # initialize memory variables 
        self.dv_eci_next = np.zeros(3)
        self.t_burn = self.dt_lqr
        
        return
    
    def dv_to_dt(self, dv_scalar):
        """approximate conversion from delta-v (m/s) to thruster firing duration (s)
        """
        return self.mass / self.thrust * dv_scalar 

    def control_callback(self, state_msg_self, state_msg_ref) -> None:
        """
        returns thrust command and attitude location
        """

        sec_int     = state_msg_self.header.stamp.sec
        nanosec_int = state_msg_self.header.stamp.nanosec
        t_stamp = sec_int + nanosec_int * 1.0E-9

        ######################################################
        # Determine if current time (t_stamp) is approximately aligned with
        # - frame to apply burn
        # - frame precompute delta-v
        ######################################################
        frame_apply_burn = np.abs(t_stamp - self.t_burn) < self.tol
        frame_precompute_dv = np.abs(t_stamp - (self.t_burn - self.time_attitude_settle)) < self.tol

        if frame_precompute_dv:
            
            # compute positions and velocities
            pos_self_eci = np.array(state_msg_self.r_bn_n)
            vel_self_eci = np.array(state_msg_self.v_bn_n)
            pos_ref_eci  = np.array(state_msg_ref.r_bn_n)
            vel_ref_eci  = np.array(state_msg_ref.v_bn_n)
            posvel_self  = np.concatenate([pos_self_eci, vel_self_eci])
            posvel_ref   = np.concatenate([pos_ref_eci, vel_ref_eci])

            # update next velocity and next attitude to change to.
            self.dv_eci_next = self.lqr.update(posvel_self, posvel_ref, self.time_attitude_settle)
            
            # compute next orientation and desired attitude pointing duration
            dt_burn_next = self.dv_to_dt(np.linalg.norm(self.dv_eci_next))
            if dt_burn_next > self.dt_lqr:
                self.get_logger().info("warning: dt_burn_next > dt_lqr")
            mrp_des = get_attitude_thrust(self.dv_eci_next, self.thruster_vec)
            
            time_b = self.t_burn - self.time_attitude_settle # attitude schedule, next
            time_e = self.t_burn + dt_burn_next
            self.att_scheduler.add_slot(AttitudeScheduler.AttMode.THRUST,
                                        time_b, time_e, mrp_des)
        
        elif frame_apply_burn:

            # print('Hello!')
            dv_now = np.linalg.norm(self.dv_eci_next)
            dt_apply_now = self.dv_to_dt(dv_now) 

            # apply current thrust command 
            thr_msg = ThrustCmd()
            thr_msg.on_time_out_msg = [0.0]*36
            thr_msg.on_time_out_msg[0] = dt_apply_now
            self.thrust_publisher.publish(thr_msg)
            self.t_burn += self.dt_lqr # increment burn time [sec]
            
            
        # check attitude commands
        slot = self.att_scheduler.get_att(t_stamp)
        if slot.data['att_mode'] == AttitudeScheduler.AttMode.THRUST:
            msg = AttGuid()
            mrp_des = slot.data['fixed_mrp_des']
            msg.sigma_br = mrp_des.tolist()
            self.att_cmd_publisher.publish(msg)

        return

    def control_callback_passthru(self, state_msg_self, state_msg_ref, att_guid_msg) -> None:
        
        
        ####################################
        # LQR part
        ####################################
        self.control_callback(state_msg_self, state_msg_ref)
        
        ####################################
        # pass thru attitude guidance
        ####################################
        sec_int     = state_msg_self.header.stamp.sec
        nanosec_int = state_msg_self.header.stamp.nanosec
        t_stamp = sec_int + nanosec_int * 1.0E-9
        
        slot = self.att_scheduler.get_att(t_stamp)
        if slot.data['att_mode'] == AttitudeScheduler.AttMode.SENSOR:
            self.att_cmd_publisher.publish(att_guid_msg)

        return
    
    
def main(args=None):
    rclpy.init(args=args)
    formation_ctrl = FormationControl()
    rclpy.spin(formation_ctrl)
    formation_ctrl.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()