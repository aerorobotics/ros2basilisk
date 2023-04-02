# https://hanspeterschaub.info/basilisk/Documentation/architecture/msgPayloadDefC/SCStatesMsgPayload.html
# most of these are vectors with (x, y ,z)

std_msgs/Header header
float64[] r_bn_n
float64[] v_bn_n
float64[] r_cn_n
float64[] v_cn_n
float64[] sigma_bn
float64[] omega_bn_b
float64[] omegadot_bn_b
float64[] totalaccumdvbdy
float64[] totalaccumdv_bn_b
float64[] nonconservativeaccelpntb_b
uint64 mrpswitchcount
