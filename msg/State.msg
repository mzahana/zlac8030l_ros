Header header

# Motor CAN node ID
int16 node_id

# Battery voltage read by the driver [V]
float32 voltage

# Electric target current [mA]
float64 target_current_mA

# Electric target current [A]
float64 target_current_A

# Motor current [A]
float32 current

# Error/Fault last code
int32 error_code

# Error code in readable string format
string error_string

# Target speed [rpm]
float64 target_speed

# Current speed [rpm]
float64 actual_speed