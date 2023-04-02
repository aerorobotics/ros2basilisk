# https://hanspeterschaub.info/basilisk/Documentation/architecture/msgPayloadDefC/AttGuidMsgPayload.html

std_msgs/Header header

# Current attitude error estimate (MRPs) of B relative to R*/.
float64[] sigma_br

# [r/s] Current body error estimate of B relateive to R in B frame compoonents
float64[] omega_br_b

# Reference frame rate vector of the of R relative to N in B frame components
float64[] omega_rn_b

# [r/s2] Reference frame inertial body acceleration of R relative to N in B frame components
float64[] domega_rn_b