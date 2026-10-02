# PX4 VehicleStatus uORB message, MESSAGE_VERSION 1.
# Keep this field layout compatible with PX4's msg/versioned/VehicleStatus.msg
# because it is used on /fmu/out/vehicle_status.

uint32 MESSAGE_VERSION = 1

uint64 timestamp
uint64 armed_time
uint64 takeoff_time

uint8 arming_state
uint8 ARMING_STATE_DISARMED = 1
uint8 ARMING_STATE_ARMED = 2

uint8 latest_arming_reason
uint8 latest_disarming_reason
uint8 ARM_DISARM_REASON_TRANSITION_TO_STANDBY = 0
uint8 ARM_DISARM_REASON_STICK_GESTURE = 1
uint8 ARM_DISARM_REASON_RC_SWITCH = 2
uint8 ARM_DISARM_REASON_COMMAND_INTERNAL = 3
uint8 ARM_DISARM_REASON_COMMAND_EXTERNAL = 4
uint8 ARM_DISARM_REASON_MISSION_START = 5
uint8 ARM_DISARM_REASON_SAFETY_BUTTON = 6
uint8 ARM_DISARM_REASON_AUTO_DISARM_LAND = 7
uint8 ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT = 8
uint8 ARM_DISARM_REASON_KILL_SWITCH = 9
uint8 ARM_DISARM_REASON_LOCKDOWN = 10
uint8 ARM_DISARM_REASON_FAILURE_DETECTOR = 11
uint8 ARM_DISARM_REASON_SHUTDOWN = 12
uint8 ARM_DISARM_REASON_UNIT_TEST = 13

uint64 nav_state_timestamp
uint8 nav_state_user_intention
uint8 nav_state
uint8 NAVIGATION_STATE_MANUAL = 0
uint8 NAVIGATION_STATE_ALTCTL = 1
uint8 NAVIGATION_STATE_POSCTL = 2
uint8 NAVIGATION_STATE_AUTO_MISSION = 3
uint8 NAVIGATION_STATE_AUTO_LOITER = 4
uint8 NAVIGATION_STATE_AUTO_RTL = 5
uint8 NAVIGATION_STATE_POSITION_SLOW = 6
uint8 NAVIGATION_STATE_FREE5 = 7
uint8 NAVIGATION_STATE_FREE4 = 8
uint8 NAVIGATION_STATE_FREE3 = 9
uint8 NAVIGATION_STATE_ACRO = 10
uint8 NAVIGATION_STATE_FREE2 = 11
uint8 NAVIGATION_STATE_DESCEND = 12
uint8 NAVIGATION_STATE_TERMINATION = 13
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15
uint8 NAVIGATION_STATE_FREE1 = 16
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17
uint8 NAVIGATION_STATE_AUTO_LAND = 18
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20
uint8 NAVIGATION_STATE_ORBIT = 21
uint8 NAVIGATION_STATE_AUTO_VTOL_TAKEOFF = 22
uint8 NAVIGATION_STATE_EXTERNAL1 = 23
uint8 NAVIGATION_STATE_EXTERNAL2 = 24
uint8 NAVIGATION_STATE_EXTERNAL3 = 25
uint8 NAVIGATION_STATE_EXTERNAL4 = 26
uint8 NAVIGATION_STATE_EXTERNAL5 = 27
uint8 NAVIGATION_STATE_EXTERNAL6 = 28
uint8 NAVIGATION_STATE_EXTERNAL7 = 29
uint8 NAVIGATION_STATE_EXTERNAL8 = 30
uint8 NAVIGATION_STATE_MAX = 31

uint8 executor_in_charge
uint32 valid_nav_states_mask
uint32 can_set_nav_states_mask

uint16 failure_detector_status
uint16 FAILURE_NONE = 0
uint16 FAILURE_ROLL = 1
uint16 FAILURE_PITCH = 2
uint16 FAILURE_ALT = 4
uint16 FAILURE_EXT = 8
uint16 FAILURE_ARM_ESC = 16
uint16 FAILURE_BATTERY = 32
uint16 FAILURE_IMBALANCED_PROP = 64
uint16 FAILURE_MOTOR = 128

uint8 hil_state
uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

uint8 vehicle_type
uint8 VEHICLE_TYPE_ROTARY_WING = 1
uint8 VEHICLE_TYPE_FIXED_WING = 2
uint8 VEHICLE_TYPE_ROVER = 3

uint8 FAILSAFE_DEFER_STATE_DISABLED = 0
uint8 FAILSAFE_DEFER_STATE_ENABLED = 1
uint8 FAILSAFE_DEFER_STATE_WOULD_FAILSAFE = 2

bool failsafe
bool failsafe_and_user_took_over
uint8 failsafe_defer_state

bool gcs_connection_lost
uint8 gcs_connection_lost_counter
bool high_latency_data_link_lost

bool is_vtol
bool is_vtol_tailsitter
bool in_transition_mode
bool in_transition_to_fw

uint8 system_type
uint8 system_id
uint8 component_id

bool safety_button_available
bool safety_off

bool power_input_valid
bool usb_connected

bool open_drone_id_system_present
bool open_drone_id_system_healthy

bool parachute_system_present
bool parachute_system_healthy

bool rc_calibration_in_progress
bool calibration_enabled
bool pre_flight_checks_pass
