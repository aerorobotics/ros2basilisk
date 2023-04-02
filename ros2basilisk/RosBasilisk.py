#!/usr/bin/env python3

from collections import deque
import numpy as np
import yaml

# ROS2 imports
import rclpy
from rclpy.node import Node

# Basilisk imports
from Basilisk.utilities import macros

# Relative imports
from ros2basilisk.Simulation import Simulation

# interfaces imports
from ros2basilisk.msg import SCStates, ThrustCmd, RWTorque


def yamlLoad(yaml_path):
    with open(yaml_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)    
    return config

def time2stamp(time_sec_float):
    """split the time represented as seconds (float) into seconds (int) and nanoseconds (int) """
    
    sec = int(time_sec_float)
    nanosec = int((time_sec_float-sec)*1E9 +0.5)

    return sec, nanosec

class RosBasilisk(Node):
    """Main ROS2 node that wraps Basilisk
    """

    def __init__(self) -> None:
        super().__init__("RosBasilisk")

        # declare ROS parameters that can be overrriden
        self.declare_parameter("workspace_dir", "/home/ubuntu/example_ws/")
        self.declare_parameter("config_yaml", "src/ros2basilisk/config/lqr_config.yaml")
        self.declare_parameter("dir_out", "data/runs/bsk_default/bsk_out")
        self.declare_parameter("delayStep", 1.0)
        self.declare_parameter("timeStep", 1.0)
        self.declare_parameter("simTime", 1000.0)

        workspace_dir = self.get_parameter('workspace_dir').get_parameter_value().string_value
        yaml_path     = self.get_parameter('config_yaml').get_parameter_value().string_value
        dir_out       = self.get_parameter('dir_out').get_parameter_value().string_value
        delay_period  = self.get_parameter("delayStep").value
        self.sim_dt   = self.get_parameter("timeStep").value
        self.simTime  = self.get_parameter("simTime").value

        config = yamlLoad(workspace_dir+yaml_path)
        bsk_config = config["bsk_config"]
        sc_configs = bsk_config["sc_configs"]

        self.agent_names = sorted(list(sc_configs.keys()))
        
        self.num_sc = len(self.agent_names)
        self.counter = 0

        # purpose of delay_period: user should ensure other ROS nodes finish running during this time
        # Create the main timer callback
        self.timer = self.create_timer(delay_period, self.timer_callback)

        #################################################
        # Create ROS Publishers/Subscribers
        #################################################

        # ground truth state publishers
        self.state_ros_publishers= {}
        for name in self.agent_names:
            self.state_ros_publishers[name] = self.create_publisher(
                SCStates, "/"+name+"/bsk_state_gt", 5 )

        # thruster ROS subscribers
        self.thrust_ros_subscribers = {}
        self.agent2thrustDeque = {}
        for name in self.agent_names:
            self.agent2thrustDeque[name] = deque() # add empty deque
            thr_listner= lambda msg : self.thrust_subscribe_callback(msg, name)
            self.thrust_ros_subscribers[name] = self.create_subscription(
                ThrustCmd, "/"+name+"/thr_cmd", thr_listner, 3 )

        # Reaction wheel (RW) torque ROS subscribers
        self.torque_ros_subscribers = {}
        self.agent2torqueDeque = {}
        for name in self.agent_names:
            self.agent2torqueDeque[name] = deque()
            torque_listner = lambda msg : self.torque_subscribe_callback(msg, name)
            self.torque_ros_subscribers[name] = self.create_subscription(
                RWTorque, "/"+name+"/rw_torque_cmd", torque_listner, 3 )
        
        #################################################
        # Initialize 
        # - Basilisk Dynamics Simulation
        # - Basilisk Bridge Task 
        #################################################

        self.bsk_sim = Simulation(bsk_config, workspace_dir + dir_out, self.sim_dt, self.simTime)

    def thrust_subscribe_callback(self, thr_msg, agent_name: str) -> None:
        """store the latest message
        """
        self.agent2thrustDeque[agent_name].append(thr_msg)
        return

    def torque_subscribe_callback(self, torque_msg, agent_name: str) -> None:
        """store the latest message
        """
        self.agent2torqueDeque[agent_name].append(torque_msg)
        return

    def timer_callback(self) -> None:
        """Propagate simulation till next time step"""

        sim_t = self.counter*self.sim_dt  
        self.counter += 1    
        if sim_t >= self.simTime:
            self.get_logger().info("Basilisk Simulation Ended. Total time = " + str(self.simTime) )
            return
        
        if np.abs(sim_t % 10) < 1.E-6:
            msg = str(sim_t) + ' / ' + str(self.simTime) + ' sec (dt = ' + str(self.sim_dt) + ')'
            self.get_logger().info(msg)
            
        
        current_time_ns = macros.sec2nano(sim_t)

        for name in self.agent_names: 
            
            # handle message queue
            num_thr_msg = len(self.agent2thrustDeque[name])
            if num_thr_msg == 0:
                self.get_logger().debug("No thrust cmd for " + name )
                continue
            elif num_thr_msg >1:
                self.get_logger().debug("More than one thrust cmd for" + name)
            while self.agent2thrustDeque[name]:
                thr_msg_ros = self.agent2thrustDeque[name].popleft()
            
            #only write the last message
            self.get_logger().debug("Applying thrust")
            self.bsk_sim.bsk_task.write_thrust_msg(name, thr_msg_ros, current_time_ns)  
        
        
        for name in self.agent_names: 
            # handle message queue
            num_rw_msg = len(self.agent2torqueDeque[name])
            if num_rw_msg == 0:
                self.get_logger().debug("No RWS cmd for " + name )
                continue
            elif num_rw_msg >1:
                self.get_logger().debug("More than one RWS cmd for" + name)
            while self.agent2torqueDeque[name]:
                torque_msg_ros = self.agent2torqueDeque[name].popleft()

            self.get_logger().debug(f"Applying RW Torque")
            self.bsk_sim.bsk_task.write_rw_torque_msg(name,torque_msg_ros,current_time_ns)
            
        # propagate simulatin up to the new sim_t
        t_next = sim_t + self.sim_dt
        self.bsk_sim.run_one_step(t_next)

        # convert to sec(int) and nanosec(int)
        sec_int, nanosec_int = time2stamp(t_next)

        # publish ROS ground truth states 
        for name in self.agent_names: 
            sc_msg_ros = self.bsk_sim.bsk_task.read_state_msg(name)
            # create timestamp based on Basilisk time
            sc_msg_ros.header.stamp.sec = sec_int
            sc_msg_ros.header.stamp.nanosec = nanosec_int

            self.state_ros_publishers[name].publish(sc_msg_ros)

        return


def main(args=None):

    rclpy.init(args=None)
    ros_bsk_node = RosBasilisk()

    try:
        rclpy.spin(ros_bsk_node)
        ros_bsk_node.bsk_sim.post_process()
        ros_bsk_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        # TODO: it has only 5 sec to save the files? what if the data gets larger
        ros_bsk_node.bsk_sim.post_process()
        ros_bsk_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()