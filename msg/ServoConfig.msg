# Torque units enum
uint8 PCT_MAX = 0
uint8 AMPS = 1
# Torque info
uint8 torque_unit
float64 torque_limit

# Vel units enum
uint8 RPM = 0
uint8 COUNTS_PER_SEC = 1
# Vel info
uint8 vel_unit
float64 vel_limit

# All distance units are in counts
float64 encoder_cpr
float64 min_position
float64 max_position