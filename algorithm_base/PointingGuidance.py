#!/usr/bin/env python3

import numpy as np

# ros imports
import rclpy
from rclpy.node import Node
from message_filters import TimeSynchronizer, Subscriber

# castor_interfaces
from ros2basilisk.msg import SCStates, AttGuid

from utility import is_key_frame
from helper_fns.utility import stamp2time
from helper_fns.AttitudeUtil import point_sensor_mat

class PointingGuidance(Node):
    """This function publishes the attitude guidance messages
    such that spacecraft will point instruments to the desired directions. 

    dt_nav controls the rate of publishing the navigation. 
    Assume: 
    - dt_nav is integer multiple of rate at which state_msg arrives (presumably from Basilisk)
    """

    def __init__(self) -> None:
        super().__init__("PointingGuidance")

        # ROS parameters
        self.declare_parameter("seed", 4751) # make sure seed doesn't match other node
        self.declare_parameter("timeRef", 0.0) # make sure seed doesn't match other node
        self.declare_parameter("timeStep", 0.2) # make sure seed doesn't match other node
        self.declare_parameter("timeTol", 1.0E-6) # make sure seed doesn't match other node
        self.declare_parameter("los_sensor_B",  [0.0, 0.0, 1.0]) # make sure seed doesn't match other node
        
        seed = self.get_parameter('seed').value
        self.rng = np.random.default_rng(seed)
        
        # load config
        self.time_ref = self.get_parameter('timeRef').value   # seconds
        self.dt_nav   = self.get_parameter('timeStep').value
        self.tol      = self.get_parameter('timeTol').value
        self.los_B    = np.array(self.get_parameter('los_sensor_B').value) # line of sight direction in the body frame
        
        # synchronized subscriber
        state_sub_self = Subscriber(self,SCStates, "/sc_self/state_nav")
        state_sub_target  = Subscriber(self,SCStates, "/sc_target/state_nav")
        self.sync = TimeSynchronizer( [state_sub_self, state_sub_target], 3 )
        self.sync.registerCallback(self.callback)
        
        # publisher
        self.guidance_publisher = self.create_publisher(AttGuid, 'sc_self/mrp_des', 3)
        
        # set permanent variables
        self.dir_up_B = self.rng.standard_normal(3) # randomly pick "up" direction
        self.dir_up_I = self.rng.standard_normal(3)
        
        # initialize memory variable
        self.counter = 0
        
        return


    def callback(self, state_msg_self, state_msg_target):
        
        # ASSUME state_gt_topic comes in exact multiple of dt_nav
        t_stamp = stamp2time(state_msg_self.header.stamp)

        flag_keyframe = is_key_frame(t_stamp, self.time_ref, self.dt_nav, self.tol)

        if flag_keyframe:
            # close enough
            
            pos_self_eci   = np.array(state_msg_self.r_bn_n)
            pos_target_eci = np.array(state_msg_target.r_bn_n)
            relpos_eci = pos_target_eci - pos_self_eci
            rot = point_sensor_mat(self.los_B, relpos_eci, self.dir_up_B, self.dir_up_I )
            
            att_msg = AttGuid()
            att_msg.header.stamp = state_msg_self.header.stamp
            att_msg.sigma_br = rot.as_mrp().tolist()
            self.guidance_publisher.publish(att_msg)
            self.counter += 1

def main(args=None):
    rclpy.init(args=args)

    # creates publisher node
    simple_nav = PointingGuidance()
    rclpy.spin(simple_nav)
    simple_nav.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()