Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# Motion state reported at the official bunker_ros boundary.
float64 linear_velocity
float64 angular_velocity

# Base state.
uint8 base_state
uint8 control_mode
uint16 fault_code
float64 battery_voltage

# Official ROS 1 bunker_msgs uses a fixed array of three states.
BunkerMotorState[3] motor_states
