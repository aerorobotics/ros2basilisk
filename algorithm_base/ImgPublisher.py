#!/usr/bin/env python3

import cv2
import os
import numpy as np

# ROS imiports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge 

from sensor_msgs.msg import Image
from ros2basilisk.msg import SCStates

from helper_fns.utility import time2stamp


class ImgPublisher(Node):
    """This node publishes images from a specified directory at specified rate.
    """

    def __init__(self) -> None:
        super().__init__('img_publisher')

        self.declare_parameter("workspace_dir", "/home/ubuntu/castor_ws/")
        self.declare_parameter("imgSrcDir", "data/images") # relative path from workspace
        self.declare_parameter("method", "SUBSCRIBER")  # options: "TIMER", "SUBSCRIBER"
        self.declare_parameter("timeStepPub", 1.0 )
        self.declare_parameter("timeRefPub", 0.0 )
        workspace_dir = self.get_parameter('workspace_dir').get_parameter_value().string_value
        imgSrc_dir    = self.get_parameter('imgSrcDir').get_parameter_value().string_value
        self.method   = self.get_parameter('method').get_parameter_value().string_value
        self.timeStep = self.get_parameter('timeStepPub').value
        self.timeRef  = self.get_parameter('timeRefPub').value
        
        self.zfill_digit = 9
        self.img_dir = os.path.join(workspace_dir, imgSrc_dir)
        self.bridge = CvBridge()
        self.counter = 0
        
        self.publisher_ = self.create_publisher(Image, '/cam0/image_raw', 10)
        
        if self.method == 'TIMER':
            # additional parameters
            self.declare_parameter("delayStep", 0.15 )
            delay        = self.get_parameter('delayStep').value
                
            self.img_fnames = sorted(os.listdir(self.img_dir))
            self.num_imgs = len(self.img_fnames)
            
            # create timer and publisher
            self.timer = self.create_timer(delay, self.timer_callback)
            
        elif self.method == 'SUBSCRIBER':
            self.subscriber = self.create_subscription(SCStates, '/sc_name/bsk_state_gt', self.sub_callback, 3)

        else:
            raise ValueError("invalid 'method' parameter is selected")
        
        return

    def timer_callback(self) -> None:

        if self.counter < self.num_imgs:
            fname = self.img_fnames[self.counter]
            img_cv = cv2.imread(self.img_dir + fname)
            img_msg = self.bridge.cv2_to_imgmsg(np.array(img_cv), 'bgr8')

            # add simulated stamps
            sec, nsec = time2stamp(self.counter*self.timeStep + self.timeRef)
            
            img_msg.header.frame_id = str(self.counter)
            img_msg.header.stamp.sec = sec
            img_msg.header.stamp.nanosec = nsec
            
            self.publisher_.publish(img_msg)
            self.counter += 1 # increases counter
        else:
            self.get_logger().info('All images have been published')
            
        return
    
    def sub_callback(self, state_msg) -> None:
        
        time = float(state_msg.header.stamp.sec) + float(state_msg.header.stamp.nanosec)/1.0E9
        
        millisec = int(time*1.0E3)
        fname = str(millisec).zfill(self.zfill_digit) + ".png"
        dir_image = os.path.join(self.img_dir, fname)

        img_cv = cv2.imread(dir_image)
        img_msg = self.bridge.cv2_to_imgmsg(np.array(img_cv), 'bgr8')
        
        img_msg.header.frame_id = str(self.counter)
        img_msg.header.stamp.sec      = state_msg.header.stamp.sec
        img_msg.header.stamp.nanosec  = state_msg.header.stamp.nanosec
        self.publisher_.publish(img_msg)
        self.counter += 1 # increases counter
        return

def main(args=None):
    rclpy.init(args=args)

    # creates publisher node
    img_publisher = ImgPublisher()
    rclpy.spin(img_publisher)
    img_publisher.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()