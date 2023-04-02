#################################################
# Reaction Wheel Torque Message
#
# Reference:
#   https://hanspeterschaub.info/basilisk/Documentation/architecture/msgPayloadDefC/ArrayMotorTorqueMsgPayload.html#_CPPv426ArrayMotorTorqueMsgPayload
# Note:
#   Motor torque in units of N-m
#   Addditionally incldue header for timestamp
#################################################

std_msgs/Header header
float64[] motor_torque
