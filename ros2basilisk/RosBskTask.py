# basilisk imports
from Basilisk.architecture import messaging as bsk_messaging
from Basilisk.utilities import simulationArchTypes

# BSK->ROS messages are imported here:
from ros2basilisk.msg import SCStates

class RosBskTask(simulationArchTypes.PythonModelClass):
    """Basilisk Task that interfaces with ROS.

    TODO: we might not need a Basilisk Module (i.e. this class) to implement
    interfaces like state_bsk_readers? But we need to specify self.moduleID...
    """

    def __init__(self, modelName, agent_names, modelActive=True, modelPriority=-1) -> None:
        super(RosBskTask, self).__init__(modelName, modelActive, modelPriority)

        self.agent_names = agent_names
        self.num_sc = len(self.agent_names)

        ############################################
        # Basilisk messaging place holders
        ############################################

        # spacecraft state        
        self.state_bsk_readers = {}
        for agent_name in self.agent_names:
            self.state_bsk_readers[agent_name] = bsk_messaging.SCStatesMsgReader()

        # thruster cmd
        self.thrust_msgs_bsk = {}
        for agent_name in self.agent_names:
            self.thrust_msgs_bsk[agent_name] = bsk_messaging.THRArrayOnTimeCmdMsg()

        # torque cmd
        self.torque_msgs_bsk = {}
        for agent_name in self.agent_names:
            self.torque_msgs_bsk[agent_name] = bsk_messaging.CmdTorqueBodyMsg()

        # rw torque cmd
        self.rw_torque_msgs_bsk = {}
        for agent_name in self.agent_names:
            self.rw_torque_msgs_bsk[agent_name] = bsk_messaging.ArrayMotorTorqueMsg()

        return

    def write_thrust_msg(self, agent_name, thr_msg_ros, current_time_ns) -> None:
            
        #only write the last message
        cmdThrustOnMsgBuffer = bsk_messaging.THRArrayOnTimeCmdMsgPayload()
        cmdThrustOnMsgBuffer.OnTimeRequest = thr_msg_ros.on_time_out_msg
        self.thrust_msgs_bsk[agent_name].write(cmdThrustOnMsgBuffer, current_time_ns, self.moduleID)

        return

    def write_torque_msg(self, agent_name, torque_msg_ros, current_time_ns) -> None:
        # Write torque command for extFT
            
        #only write the last message
        torqueOutMsgBuffer = bsk_messaging.CmdTorqueBodyMsgPayload()
        torqueOutMsgBuffer.torqueRequestBody = torque_msg_ros.torque_request_body
        self.torque_msgs_bsk[agent_name].write(torqueOutMsgBuffer, current_time_ns, self.moduleID)
        return

    def write_rw_torque_msg(self, agent_name, rw_torque_msg_ros, current_time_ns) -> None:
        # Write torque command for RW
            
        #only write the last message
        rwTorqueOutMsgBuffer = bsk_messaging.ArrayMotorTorqueMsgPayload()
        rwTorqueOutMsgBuffer.motorTorque = rw_torque_msg_ros.motor_torque
        self.rw_torque_msgs_bsk[agent_name].write(rwTorqueOutMsgBuffer, current_time_ns, self.moduleID)
        
        return

    def read_state_msg(self, agent_name):
        """Read Basilisk SCState messages and output ROS SCState Message
        """
        
        sc_msg_bsk = self.state_bsk_readers[agent_name]() # read Basilisk message 

        ros_msg = SCStates() # create ROs msg
        ros_msg.r_bn_n = sc_msg_bsk.r_BN_N
        ros_msg.v_bn_n = sc_msg_bsk.v_BN_N
        ros_msg.r_cn_n = sc_msg_bsk.r_CN_N
        ros_msg.v_cn_n = sc_msg_bsk.v_CN_N
        ros_msg.sigma_bn = sc_msg_bsk.sigma_BN
        ros_msg.omega_bn_b = sc_msg_bsk.omega_BN_B
        ros_msg.omegadot_bn_b = sc_msg_bsk.omegaDot_BN_B
        ros_msg.totalaccumdvbdy = sc_msg_bsk.TotalAccumDVBdy
        ros_msg.totalaccumdv_bn_b = sc_msg_bsk.TotalAccumDV_BN_B
        ros_msg.nonconservativeaccelpntb_b = sc_msg_bsk.nonConservativeAccelpntB_B
        ros_msg.mrpswitchcount = sc_msg_bsk.MRPSwitchCount

        return ros_msg