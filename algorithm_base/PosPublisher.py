#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from ros2basilisk.msg import PointTimed, SCStates

from helper_fns.utility import jsonLoad, stamp2time, time2stamp
from utility import is_key_frame

class PosPublisher(Node):

    def __init__(self):
        super().__init__('gps_publisher')

        # ROS parameters
        self.declare_parameter("agent_name", "sc_name")
        self.declare_parameter("seed", 879) # make sure seed doesn't match other node
        self.declare_parameter("source", "FILES") # options: SIM, FILES
        self.declare_parameter("enableNoise", False) 
        self.declare_parameter("posSigma", 0.0) # float
        self.declare_parameter('timeStepPub', 1.0)
        self.declare_parameter('timeRefPub', 0.0)

        self.agent_name  = self.get_parameter('agent_name').get_parameter_value().string_value
        gt_source        = self.get_parameter('source').get_parameter_value().string_value    
        self.enableNoise = self.get_parameter('enableNoise').value      
        self.posSigma    = self.get_parameter('posSigma').value 
        seed = self.get_parameter('seed').value
        self.timeStep = self.get_parameter('timeStepPub').value
        self.timeRef = self.get_parameter('timeRefPub').value

        # creates a publisher which publishes PointTimed messages to the topic 
        # 'agent0/gps_meas_eci' with a queue size of 10 messages
        self.meas_publisher = self.create_publisher(PointTimed, '/sc_name/pos_eci_meas', 10)
        self.counter = 0
        self.rng = np.random.default_rng(seed)

        if gt_source == "FILES":

            # load ground truth trajectory data
            # ASSUME: 
            # - 0-th imgs corresponds to t = 0.0
            ##########################################################
            self.declare_parameter("workspace_dir", "/home/ubuntu/castor_ws/")
            self.declare_parameter("trajJson", "data/runs/ekf_default/traj.json") # path to JSON File
            self.declare_parameter("delayStep", 1.0)
            self.declare_parameter("sim_dt", 1.0)
            self.declare_parameter("timeRef", 0.0)
            
            workspace_dir = self.get_parameter('workspace_dir').get_parameter_value().string_value
            trajJson = workspace_dir + self.get_parameter('trajJson').get_parameter_value().string_value
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

    def timer_callback(self) -> None:

        if self.counter < self.num_steps:

            current_ts = self.tvec_list[self.counter]
            pos_eci = self.data[self.agent_name][current_ts]['pose_eci'][3:6]

            if self.enableNoise:
                pos_eci += self.rng.standard_normal(3)*self.posSigma

            # sets gps message values
            gps = PointTimed()
            gps.x = pos_eci[0]
            gps.y = pos_eci[1]
            gps.z = pos_eci[2]
            sec, nsec = time2stamp(self.counter*self.sim_dt + self.t_ref)
            gps.header.stamp.sec = sec
            gps.header.stamp.nanosec = nsec
            gps.header.frame_id = "ECI"

            # publishes and logs current position
            self.meas_publisher.publish(gps)
            self.counter += 1
            
        else:
            self.get_logger().info('All GPS measurements have been published')
        return
            
    def sub_callback(self, state_msg) -> None:
        
        t_now = stamp2time(state_msg.header.stamp)
        if not is_key_frame(t_now, self.timeRef, self.timeStep, 1.0E-6):
            return # skip if not keyframe

        pos_eci = np.array(state_msg.r_bn_n)
        if self.enableNoise:
            pos_eci += self.rng.standard_normal(3)*self.posSigma
    
        gps = PointTimed()
        gps.x = pos_eci[0]
        gps.y = pos_eci[1]
        gps.z = pos_eci[2]
        gps.header = state_msg.header # copy the header

        # publishes and logs current position
        self.meas_publisher.publish(gps)
        self.counter += 1
        
        return
        
def main(args=None):
    rclpy.init(args=args)
    gps_publisher = PosPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()