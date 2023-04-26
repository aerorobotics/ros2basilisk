#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from ros2basilisk.msg import SCStates

from helper_fns.utility import jsonLoad, time2stamp, stamp2time
from utility import is_key_frame

class AttPublisher(Node):

    def __init__(self):
        super().__init__('ori_publisher')

        self.declare_parameter("agent_name", "sc_name")
        self.declare_parameter("seed", 172) # make sure seed doesn't collide with other seed
        self.declare_parameter("source", "SIM") # options: SIM, FILES
        self.declare_parameter("enableNoise", False) 
        self.declare_parameter("includeRate", False) 
        self.declare_parameter("attSigma", 0.0) # float
        self.declare_parameter('timeStepPub', 1.0) # [sec] rate of attitude publisher
        self.declare_parameter('timeRefPub', 0.0)
        
        self.agent_name  = self.get_parameter('agent_name').get_parameter_value().string_value
        gt_source        = self.get_parameter('source').get_parameter_value().string_value    
        self.enableNoise = self.get_parameter('enableNoise').value    
        self.includeRate = self.get_parameter('includeRate').value    
        self.attSigma    = self.get_parameter('attSigma').value 
        seed             = self.get_parameter('seed').value
        self.timeStep    = self.get_parameter('timeStepPub').value
        self.timeRef     = self.get_parameter('timeRefPub').value
         
        self.meas_publisher = self.create_publisher(SCStates, '/sc_name/att_eci_meas', 10)
        self.counter = 0
        self.rng = np.random.default_rng(seed)

        if gt_source == "FILES":

            # load ground truth trajectory data
            # ASSUME: 
            # - 0-th imgs corresponds to t = 0.0
            ##########################################################
            
            self.declare_parameter("trajJson", "/home/ubuntu/castor_ws/data/runs/ekf_default/traj.json") # path to JSON File
            self.declare_parameter("delayStep", 1.0)
            self.declare_parameter("sim_dt", 1.0)
            self.declare_parameter("timeRef", 0.0)
            
            trajJson = self.get_parameter('trajJson').get_parameter_value().string_value
            timer_period = self.get_parameter('delayStep').value 
            self.sim_dt  = self.get_parameter("sim_dt").value
            self.t_ref   = self.get_parameter("timeRef").value
            
            # load data
            self.data = jsonLoad(trajJson)
            self.tvec_list = sorted(list(self.data[self.agent_name].keys()))
            self.num_steps = len(self.tvec_list)

            # creates a timer with a callback to execute every 1 second
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
        elif gt_source == "SIM":
            
            self.state_sub = self.create_subscription(SCStates, '/sc_name/bsk_state_gt', self.sub_callback, 3)
            
        return


    def timer_callback(self):
        # opens and loads file

        if self.counter < self.num_steps:

            current_ts = self.tvec_list[self.counter]
            pose_eci = self.data[self.agent_name][current_ts]['pose_eci']

            # CAUTION: trajectory file uses rotation vector convention for attitude representation.
            rot_c_eci = R.from_rotvec([pose_eci[0], pose_eci[1], pose_eci[2]])
            if self.enableNoise:
                delta_mpr = self.rng.standard_normal(3)*self.attSigma
                rot_perturb = R.from_rotvec(delta_mpr)
                rot_c_eci = rot_c_eci * rot_perturb # add perturbation
                
            # converts 3 element rotation object to 4 element quaternion object
            mrp_cam = rot_c_eci.as_mrp()

            # sets orientation message values
            ori = SCStates()
            ori.sigma_bn = mrp_cam.tolist()
            if self.includeRate:
                raise NotImplementedError("ground truth JSON file must contain angular rate...")

            # add simulated stamps
            sec, nsec = time2stamp(self.counter * self.sim_dt + self.t_ref)
            ori.header.stamp.sec = sec
            ori.header.stamp.nanosec = nsec
            ori.header.frame_id = "ECI"

            # publishes and logs current orientation
            self.meas_publisher.publish(ori)
            self.counter += 1
        else:
            self.get_logger().info('All attitude measurements have been published')
        

    def sub_callback(self, state_msg) -> None:
        
        t_now = stamp2time(state_msg.header.stamp)
        if not is_key_frame(t_now, self.timeRef, self.timeStep, 1.0E-6):
            return # skip if not keyframe

        # CAUTION: state message uses MRP convention for attitude representation.
        rot_c_eci = R.from_mrp(np.array(state_msg.sigma_bn))
        
        if self.enableNoise:
            delta_mpr = self.rng.standard_normal(3)*self.attSigma
            rot_perturb = R.from_rotvec(delta_mpr)
            rot_c_eci = rot_c_eci * rot_perturb # add perturbation
        
        mrp_cam = rot_c_eci.as_mrp()
        
        # sets orientation message values
        ori = SCStates()
        ori.sigma_bn = mrp_cam.tolist()
        if self.includeRate:
            # TODO add perturbation
            ori.omega_bn_b = state_msg.omega_bn_b
            
        ori.header = state_msg.header


        # publishes and logs current position
        self.meas_publisher.publish(ori)
        self.counter += 1
        
        return
    
def main(args=None):
    rclpy.init(args=args)
    att_publisher = AttPublisher()
    rclpy.spin(att_publisher)
    att_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()


if __name__ == '__main__':
    main()