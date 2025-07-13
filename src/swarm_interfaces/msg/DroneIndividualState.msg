# Represents the state of a single drone in the swarm
uint8 drone_id          # Unique ID for the drone (e.g., 1, 2, 3)
string drone_ns         # ROS Namespace of the drone (e.g., px4_1)

# Local position information
geometry_msgs/Point position    # Local position (x, y, z) in meters
geometry_msgs/Vector3 velocity  # Local velocity (vx, vy, vz) in m/s
float32 heading_rad             # Heading in radians (yaw)

# Global position information
float64 latitude          # Latitude in degrees (WGS84)
float64 longitude         # Longitude in degrees (WGS84)
float32 altitude          # Altitude in meters (AMSL)
bool global_position_valid # Whether the global position is valid

# Status information
bool armed                      # True if armed, false otherwise
uint8 flight_mode_raw           # Raw flight mode from PX4 (VehicleStatus.nav_state)
string flight_mode_str          # Human-readable flight mode

# Enhanced status information
string operational_mode     # Drone's current operational state (IDLE, KALKIS_YAPIYOR, GOREV_UCUSU, FORMASYONDA, INIS_YAPIYOR, HATA_DURUMU)
float32 battery_percentage  # Remaining battery percentage (0-100)
uint8 gps_fix_type          # GPS fix type (0: no fix, 1: time only, 2: 2D, 3: 3D, 4: DGPS, 5: RTK float, 6: RTK fixed)
uint8 num_satellites        # Number of GPS satellites being used
float32 estimation_confidence # State estimation confidence (0.0-1.0)
string local_planner_status # Status of local path planner (ENGEL_YOK, KACINMA_AKTIF, etc.)
string assigned_role        # Role assigned by orchestrator ("lider", "follower", etc.)

# Timestamps
builtin_interfaces/Time last_local_pos_timestamp
builtin_interfaces/Time last_attitude_timestamp
builtin_interfaces/Time last_status_timestamp
builtin_interfaces/Time last_global_pos_timestamp