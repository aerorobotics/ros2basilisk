#!/usr/bin/env python3

import numpy as np

# ros imports
import rclpy
from rclpy.node import Node

# ros2 interfaces
from ros2basilisk.msg import SCStates

# custom utility
from utility import is_key_frame, copy_SCStates
from helper_fns.utility import yamlLoad, stamp2time

class SimpleNav(Node):
    """This function publishes the "estimated" spacecraft state based on the ground truth

    dt_nav controls the rate of publishing the navigation. 
    Assume: 
    - dt_nav is integer multiple of rate at which state_gt_msg arrives (presumably from Basilisk)
    """

    def __init__(self) -> None:
        super().__init__("SimpleNav")

        # load parameters
        self.declare_parameter("timeRef", 0.0) # make sure seed doesn't match other node
        self.declare_parameter("timeStep", 0.2) # make sure seed doesn't match other node
        self.declare_parameter("timeTol", 1.0E-6) # make sure seed doesn't match other node
        
        self.time_ref = self.get_parameter('timeRef').value   # seconds
        self.dt_nav   = self.get_parameter('timeStep').value
        self.tol      = self.get_parameter('timeTol').value
        self.counter = 0


        # subscriber / publisher
        self.state_gt_subscriber = self.create_subscription(SCStates, 'sc_name/bsk_state_gt', self.callback, 3)
        self.state_est_publihser = self.create_publisher(SCStates, 'sc_name/state_nav', 3)


    def callback(self, state_gt_msg):
        
        # ASSUME state_gt_topic comes in exact multiple of dt_nav
        t_stamp = stamp2time(state_gt_msg.header.stamp)

        flag_keyframe = is_key_frame(t_stamp, self.time_ref, self.dt_nav, self.tol)

        if flag_keyframe:
            # close enough
            state_est_msg = copy_SCStates(state_gt_msg) # copy msg
            self.state_est_publihser.publish(state_est_msg)
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    # creates publisher node
    simple_nav = SimpleNav()
    rclpy.spin(simple_nav)
    simple_nav.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()