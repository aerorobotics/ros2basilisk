#################################################
# extForce Torque Message
#
# Reference:
#   https://hanspeterschaub.info/basilisk/Documentation/architecture/msgPayloadDefC/CmdTorqueBodyMsgPayload.html
# Note:
#   torque_request_body in units of N-m
#   Addditionally incldue header for timestamp
#################################################

std_msgs/Header header
float64[] torque_request_body
