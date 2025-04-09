# Custom message for control of Kuka Robot

# Array of float64 commands
float64[] commands

# Timestamp of the message
builtin_interfaces/Time timestamp

# Integer for control method (1 = position, 2 = velocity, 3 = torque)
int32 control_method

