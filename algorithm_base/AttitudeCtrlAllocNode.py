#!/usr/bin/env python3

import numpy as np
import os

# ros imports
import rclpy
from rclpy.node import Node

# ros2 interfaces
from ros2basilisk.msg import AttGuid, SCStates, RWTorque

from helper_fns.attitude_ctrl import mrp_control, rws_alloc
from helper_fns.utility import jsonDump, yamlLoad, stamp2time

class MrpAttitudeControlAlloc(Node):
    """ROS node for MRP controller and Reaction Wheel control allocator.
        Input:  AttGuid (MPR)
        Output: ExtTorque (Nm)
    """
    
    def __init__(self)->None:
        super().__init__("MrpAttitudeControlAlloc")

        # declare ROS parameters that can be overrriden        
        self.declare_parameter('sc_name', 'agent0')
        self.declare_parameter("workspace_dir", "/home/ubuntu/example_ws/")
        self.declare_parameter("config_yaml", "src/ros2basilisk/config/lqr_config.yaml")
        self.declare_parameter("Kp", 10.0)
        self.declare_parameter("Kd", 100.0)
        self.declare_parameter("save_debug", False)
        self.declare_parameter("output_dir", "/home/ubuntu/example_ws/data/runs/lqr_default/mrp_ctrl/")
        
        # load ADCS config file
        self.sc_name = self.get_parameter('sc_name').get_parameter_value().string_value
        ws = self.get_parameter('workspace_dir').get_parameter_value().string_value
        yaml_path = self.get_parameter('config_yaml').get_parameter_value().string_value
        config    = yamlLoad(ws + yaml_path)
        adcs_config = config["adcs_config"][self.sc_name]
        
        Kp = self.get_parameter('Kp').value
        Kd = self.get_parameter('Kd').value
        
        self.save_debug = self.get_parameter('save_debug').value
        
        # define the attiude controller and control allocator
        self.controller = mrp_control(Kp, Kd)
        self.alloc      = rws_alloc(adcs_config['reaction_wheels'])

        # subscribe to desired attitude
        self.att_cmd_subscriber = self.create_subscription(
            AttGuid, '/sc_name/mrp_des', self.desired_att_update_callback, 3)
        self.state_subscriber = self.create_subscription(
            SCStates, '/sc_name/state_nav', self.control_callback, 3)
        self.rw_torque_publisher = self.create_publisher(
            RWTorque, '/sc_name/rw_torque_cmd', 10)

        self.att_des = None # initialize desired attitude as identity
        
        if self.save_debug:
            self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
            if not os.path.exists(self.output_dir):
                self.get_logger().info("MRP control debug output directory does not exist. Creating: " + self.output_dir)
                os.mkdir(self.output_dir)
                
            self.out = {} 
            self.out['tvec'] = []
            self.out['AttDes'] = []
            self.out['Torque'] = []

        return

    def desired_att_update_callback(self, att_cmd_msg) -> None:
        """update the desired attitude as soon as you receive the message"""
        self.att_des = att_cmd_msg.sigma_br
        return
    
    def control_callback(self, state_msg) -> None:
        """Upon receiving new state estimate, compute torque command and publish
        """
        
        if self.att_des is None:
            return # attitude target is not initialized yet
        
        # compute overall torque required in body frame
        torque_b = self.controller.update(state_msg.sigma_bn, state_msg.omega_bn_b, self.att_des)
        
        # convert overall torque to wheel torques
        rw_torque_list = self.alloc.allocate(torque_b)
        rw_torque_cmd_msg = RWTorque()
        rw_torque_cmd_msg.motor_torque = rw_torque_list
        self.rw_torque_publisher.publish(rw_torque_cmd_msg)
        
        if self.save_debug:
            t_now = stamp2time(state_msg.header.stamp)
            self.out['tvec'].append(t_now)
            self.out['AttDes'].append(self.att_des.tolist())
            self.out['Torque'].append(rw_torque_list)
            
        return

def main(args=None):
    rclpy.init(args=args)

    # creates publisher node
    att_ctrl = MrpAttitudeControlAlloc()
    try:
        rclpy.spin(att_ctrl)
    except KeyboardInterrupt:
        if att_ctrl.save_debug:
            jsonDump(att_ctrl.out, att_ctrl.output_dir+"mrp_ctrl_alloc.json")
        att_ctrl.destroy_node() # Destroy the node explicitly
        rclpy.shutdown()

if __name__ == '__main__':
    main()