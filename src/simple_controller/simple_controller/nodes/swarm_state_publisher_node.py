import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading

from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus as Px4VehicleStatus, VehicleGlobalPosition, BatteryStatus
from lifecycle_msgs.msg import State as LifecycleState_msg

from swarm_interfaces.msg import DroneIndividualState, SwarmState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header, String
from rclpy.executors import SingleThreadedExecutor
import math
import time
import builtin_interfaces.msg
from builtin_interfaces.msg import Time

PX4_NAV_STATE_MANUAL = 0
PX4_NAV_STATE_ALTCTL = 1
PX4_NAV_STATE_POSCTL = 2
PX4_NAV_STATE_AUTO_MISSION = 3
PX4_NAV_STATE_AUTO_LOITER = 4
PX4_NAV_STATE_AUTO_RTL = 5
PX4_NAV_STATE_ACRO = 6
PX4_NAV_STATE_OFFBOARD = 7
PX4_NAV_STATE_STAB = 8
PX4_NAV_STATE_PRECISION_LAND = 9
PX4_NAV_STATE_ORBIT = 10
PX4_NAV_STATE_AUTO_TAKEOFF = 11
PX4_NAV_STATE_AUTO_LAND = 12
PX4_NAV_STATE_AUTO_FOLLOW_TARGET = 13
PX4_NAV_STATE_AUTO_PRECLAND = 14  
PX4_NAV_STATE_TERMINATION = 16
PX4_NAV_STATE_FAILSAFE = 17
PX4_NAV_STATE_OFFBOARD_ALT = 20  

NAV_STATE_MAP = {
    PX4_NAV_STATE_MANUAL: "MANUAL",
    PX4_NAV_STATE_ALTCTL: "ALTITUDE_CONTROL",
    PX4_NAV_STATE_POSCTL: "POSITION_CONTROL",
    PX4_NAV_STATE_AUTO_MISSION: "AUTO_MISSION",
    PX4_NAV_STATE_AUTO_LOITER: "AUTO_LOITER",
    PX4_NAV_STATE_AUTO_RTL: "RETURN_TO_LAUNCH",
    PX4_NAV_STATE_ACRO: "ACRO",
    PX4_NAV_STATE_OFFBOARD: "OFFBOARD",
    PX4_NAV_STATE_STAB: "STABILIZED",
    PX4_NAV_STATE_PRECISION_LAND: "PRECISION_LAND",
    PX4_NAV_STATE_ORBIT: "ORBIT",
    PX4_NAV_STATE_AUTO_TAKEOFF: "AUTO_TAKEOFF",
    PX4_NAV_STATE_AUTO_LAND: "AUTO_LAND",
    PX4_NAV_STATE_AUTO_FOLLOW_TARGET: "FOLLOW_TARGET",
    PX4_NAV_STATE_AUTO_PRECLAND: "PRECISION_LANDING",
    PX4_NAV_STATE_TERMINATION: "TERMINATION",
    PX4_NAV_STATE_FAILSAFE: "FAILSAFE",
    PX4_NAV_STATE_OFFBOARD_ALT: "OFFBOARD", 
}

 # Dron hangi yöne bakıyor, response radian
def quaternion_to_euler_yaw(q_x, q_y, q_z, q_w):
    """Convert a quaternion into yaw (extrinsic ZYX rotation)."""
    t3 = +2.0 * (q_w * q_z + q_x * q_y)
    t4 = +1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z  # in radians

class SwarmStatePublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__('swarm_state_publisher_node')

        # Add thread lock for data protection
        self._lock = threading.RLock()

        self.declare_parameter('drone_namespaces', ['px4_1', 'px4_2', 'px4_3']) # Default to include all available drones
        self.declare_parameter('publish_frequency', 5.0) # Hz - CPU tasarrufu için 10Hz'den 5Hz'e düşürüldü
        self.declare_parameter('position_timeout_ms', 500.0) # Position data timeout threshold in milliseconds
        self.declare_parameter('attitude_timeout_ms', 500.0) # Attitude data timeout threshold in milliseconds
        self.declare_parameter('status_timeout_ms', 1000.0) # Status data timeout threshold in milliseconds
        self.declare_parameter('global_pos_timeout_ms', 2000.0) # Global position data timeout threshold in milliseconds
        
   
        # This will be done in the on_configure callback
        # key:px4_1 value: DroneIndividualState object  (position, velocity, heading, etc. )
        self.drone_data_map = {} #her bir dronun bireysel durumunu (DroneIndividualState object) saklamak için kullanılacak dict
        
        # Store subscribers for cleanup
        self.subscribers = [] # LifecycleNode, kendi yönetecek fakat referans tutmakta zarar olmaz
        self.publish_timer = None
        self.swarm_state_publisher = None
        self.publish_period = None
        
        # Timestamp tracking for data freshness
        self.last_received_pos_time = {}  # Map from namespace to ROS Time when last local position was received
        self.last_received_att_time = {}  # Map from namespace to ROS Time when last attitude was received
        self.last_received_sts_time = {}  # Map from namespace to ROS Time when last status was received
        self.last_received_gps_time = {}  # Map from namespace to ROS Time when last global position was received
        self.last_received_battery_time = {}  # Map from namespace to ROS Time when last battery status was received
        self.last_received_gps_info_time = {}  # Map from namespace to ROS Time when last GPS info was received
        
        self._configured_drone_namespaces = []
        
        # Default values for the new fields
        self.operational_modes = {}
        self.local_planner_statuses = {}
        self.assigned_roles = {}
        
        # Additional parameters for timeouts
        self.declare_parameter('battery_timeout_ms', 1000.0)  # Battery data timeout threshold in milliseconds
        self.declare_parameter('gps_info_timeout_ms', 1000.0)  # GPS info data timeout threshold in milliseconds

    def local_position_callback(self, msg: VehicleLocalPosition, namespace: str):
        try:
            current_ros_time = self.get_clock().now()
            
            with self._lock:
                if namespace not in self.drone_data_map:
                    if namespace not in self._configured_drone_namespaces:
                        self.get_logger().warn(
                            f"Received local_position from unexpected namespace '{namespace}'. Ignoring."
                        )
                        return
                    else:
                        self.get_logger().error(
                            f"Namespace '{namespace}' is configured but was not found in drone_data_map. "
                        )
                        return # drone_data_map'te olmayan bir drone'un verisini işlememek icin

                
                # Update position data
                drone_state = self.drone_data_map[namespace]
                drone_state.position.x = msg.x
                drone_state.position.y = msg.y
                drone_state.position.z = msg.z
                drone_state.velocity.x = msg.vx
                drone_state.velocity.y = msg.vy
                drone_state.velocity.z = msg.vz
                
                seconds, remaining_microseconds = divmod(msg.timestamp, 1000000)
                drone_state.last_local_pos_timestamp = Time(
                    sec=int(seconds),
                    nanosec=int(remaining_microseconds * 1000)
                )
                self.last_received_pos_time[namespace] = current_ros_time
        except Exception as e:
            self.get_logger().error(f"Error in local_position_callback for {namespace}: {str(e)}")


    def attitude_callback(self, msg: VehicleAttitude, drone_ns: str):  ## heading açısı
        try:
            current_ros_time = self.get_clock().now()
            with self._lock:
                if drone_ns in self.drone_data_map:
                    drone_state = self.drone_data_map[drone_ns]
                    drone_state.heading_rad = quaternion_to_euler_yaw(
                        q_x=msg.q[1], q_y=msg.q[2], q_z=msg.q[3], q_w=msg.q[0]
                    )
                    # Convert PX4 microsecond timestamp to ROS Time message
                    # buradaki epoch PX4'te cihazın açıldığı andan itibaren geçen süredir.
                    # ROS sistemi genelindeki wall clock time veya Unix epoch zamanı değildir.
                    seconds, rem_micro = divmod(msg.timestamp_sample, 1000000)
                    drone_state.last_attitude_timestamp = Time(sec=int(seconds), nanosec=int(rem_micro * 1000))
                    self.last_received_att_time[drone_ns] = current_ros_time
                elif drone_ns in self._configured_drone_namespaces: 
                     self.get_logger().debug(f"Attitude data for '{drone_ns}' received, but its entry in drone_data_map is missing.", throttle_duration_sec=5)
                else: 
                     self.get_logger().debug(f"Attitude data for '{drone_ns}' received, but it is not configured.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in attitude_callback for {drone_ns}: {str(e)}")


    def status_callback(self, msg: Px4VehicleStatus, drone_ns: str):
        try:
            current_ros_time = self.get_clock().now()
            with self._lock:
                if drone_ns in self.drone_data_map:
                    drone_state = self.drone_data_map[drone_ns]
                    drone_state.armed = msg.arming_state == Px4VehicleStatus.ARMING_STATE_ARMED
                    drone_state.flight_mode_raw = msg.nav_state
                    # Convert flight mode enum to string
                    drone_state.flight_mode_str = NAV_STATE_MAP.get(msg.nav_state, "UNKNOWN_MODE")
                    
                    seconds, rem_micro = divmod(msg.timestamp, 1000000)
                    drone_state.last_status_timestamp = Time(sec=int(seconds), nanosec=int(rem_micro * 1000))
                    self.last_received_sts_time[drone_ns] = current_ros_time
                elif drone_ns in self._configured_drone_namespaces: 
                     self.get_logger().debug(f"Status data for '{drone_ns}' received, but its entry in drone_data_map is missing.", throttle_duration_sec=5)
                else: 
                     self.get_logger().debug(f"Status data for '{drone_ns}' received, but it is not configured.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in status_callback for {drone_ns}: {str(e)}")

    def global_position_callback(self, msg: VehicleGlobalPosition, drone_ns: str):
        try:
            current_ros_time = self.get_clock().now()
            with self._lock:
                if drone_ns in self.drone_data_map:
                    drone_state = self.drone_data_map[drone_ns]
                    drone_state.latitude = msg.lat
                    drone_state.longitude = msg.lon
                    drone_state.altitude = msg.alt
                    drone_state.global_position_valid = msg.lat_lon_valid and msg.alt_valid
                    
                    seconds, rem_micro = divmod(msg.timestamp_sample, 1000000)
                    drone_state.last_global_pos_timestamp = Time(sec=int(seconds), nanosec=int(rem_micro * 1000))
                    self.last_received_gps_time[drone_ns] = current_ros_time
                elif drone_ns in self._configured_drone_namespaces: # Bilinmeyen ama konfigüre edilmişse logla
                     self.get_logger().debug(f"GPS data for '{drone_ns}' received, but its entry in drone_data_map is missing.", throttle_duration_sec=5)
                else: # Bilinmeyen ve konfigüre edilmemişse logla
                     self.get_logger().debug(f"GPS data for '{drone_ns}' received, but it is not configured.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in global_position_callback for {drone_ns}: {str(e)}")
            
    def battery_status_callback(self, msg: BatteryStatus, drone_ns: str):
        """
        Callback for battery status updates from PX4
        """
        try:
            current_ros_time = self.get_clock().now()
            with self._lock:
                if drone_ns in self.drone_data_map:
                    # Update drone state with battery info
                    drone_state = self.drone_data_map[drone_ns]
                    drone_state.battery_percentage = msg.remaining * 100.0  # Convert to percentage
                    
                    # Very simple operational mode determination based on battery
                    # In a real system, this would be part of a state machine
                    if msg.remaining < 0.2:  # 20% battery
                        self.operational_modes[drone_ns] = "HATA_DURUMU"
                        self.get_logger().warn(f"Drone {drone_ns} has low battery: {msg.remaining * 100.0:.1f}%")
                    
                    # Update timestamp
                    self.last_received_battery_time[drone_ns] = current_ros_time
                elif drone_ns in self._configured_drone_namespaces:
                    self.get_logger().debug(f"Battery data for '{drone_ns}' received, but its entry in drone_data_map is missing.", throttle_duration_sec=5)
                else:
                    self.get_logger().debug(f"Battery data for '{drone_ns}' received, but it is not configured.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in battery_status_callback for {drone_ns}: {str(e)}")
            
    def gps_info_callback(self, msg: VehicleGlobalPosition, drone_ns: str):
        """
        Callback for GPS status updates from PX4 using VehicleGlobalPosition message
        """
        try:
            current_ros_time = self.get_clock().now()
            with self._lock:
                if drone_ns in self.drone_data_map:
                    # Update drone state with GPS info
                    drone_state = self.drone_data_map[drone_ns]
                    
                    # VehicleGlobalPosition doesn't have fix_type or satellites_used directly
                    # Use alternative fields to determine GPS quality
                    
                    # Default values for fields we don't have direct access to
                    fix_type = 0  # Default: no fix
                    num_satellites = 0  # Unknown count
                    
                    # If lat/lon is valid, assume we have at least a 3D fix
                    if msg.lat_lon_valid:
                        fix_type = 3  # 3D fix
                        # Higher number indicates better position
                        num_satellites = max(4, int(10.0 / (msg.eph + 0.1))) if msg.eph > 0 else 10
                    
                    drone_state.gps_fix_type = fix_type
                    drone_state.num_satellites = num_satellites
                    
                    # Calculate estimation confidence based on position error metrics
                    # Lower eph/epv (error values) means higher confidence
                    if not msg.lat_lon_valid:
                        drone_state.estimation_confidence = 0.0  # No fix
                    else:
                        # Use horizontal position error (eph) to estimate confidence
                        # Lower eph means higher confidence (capped at 20m)
                        h_confidence = max(0.0, 1.0 - (min(msg.eph, 20.0) / 20.0))
                        
                        # Use vertical position error (epv) for altitude confidence
                        # Lower epv means higher confidence (capped at 20m)
                        v_confidence = max(0.0, 1.0 - (min(msg.epv, 20.0) / 20.0)) if msg.alt_valid else 0.0
                        
                        # Combine horizontal and vertical confidence, weighting horizontal more
                        drone_state.estimation_confidence = (0.7 * h_confidence) + (0.3 * v_confidence)
                    
                    # Update timestamp
                    self.last_received_gps_info_time[drone_ns] = current_ros_time
                elif drone_ns in self._configured_drone_namespaces:
                    self.get_logger().debug(f"GPS info for '{drone_ns}' received, but its entry in drone_data_map is missing.", throttle_duration_sec=5)
                else:
                    self.get_logger().debug(f"GPS info for '{drone_ns}' received, but it is not configured.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in gps_info_callback for {drone_ns}: {str(e)}")

    def publish_swarm_state(self):
        if not self.swarm_state_publisher or self._state_machine.current_state[0] != LifecycleState_msg.PRIMARY_STATE_ACTIVE:
            return
        
        try:
            # Create SwarmState message and initialize headers
            current_time = self.get_clock().now()
            swarm_state_msg = SwarmState()
            swarm_state_msg.header.stamp = current_time.to_msg()
            swarm_state_msg.header.frame_id = "map" # We're using a global reference frame
            
            # Variables to calculate swarm centroid
            valid_positions_count = 0
            sum_lat = 0.0
            sum_lon = 0.0
            sum_alt = 0.0
            sum_x = 0.0
            sum_y = 0.0
            
            # Log çıktısını DEBUG seviyesinde ve sadece gerektiğinde oluşturacağız
            log_swarm_state = self.get_logger().get_effective_level() <= 10  # DEBUG seviyesinde mi?
            log_output = ["\n--- Swarm State ---"] if log_swarm_state else None
            
            # For each drone in our data map, add to the SwarmState message if data is valid
            with self._lock:
                for ns, drone_data in self.drone_data_map.items():
                    # Get our timeout thresholds from parameters
                    position_timeout_ms = self.get_parameter('position_timeout_ms').get_parameter_value().double_value
                    attitude_timeout_ms = self.get_parameter('attitude_timeout_ms').get_parameter_value().double_value
                    status_timeout_ms = self.get_parameter('status_timeout_ms').get_parameter_value().double_value
                    global_pos_timeout_ms = self.get_parameter('global_pos_timeout_ms').get_parameter_value().double_value
                    battery_timeout_ms = self.get_parameter('battery_timeout_ms').get_parameter_value().double_value
                    gps_info_timeout_ms = self.get_parameter('gps_info_timeout_ms').get_parameter_value().double_value
                    
                    # Zaman hesaplamasını optimize ediyoruz - sadece gereken zaman farklarını hesaplayıp
                    # float('inf') yerine sabit bir büyük değer kullanacağız
                    BIG_TIMEOUT = 9999999.0
                    
                    # Calculate age of data streams
                    pos_age_ms = BIG_TIMEOUT
                    if ns in self.last_received_pos_time:
                        pos_age_ms = (current_time.nanoseconds - self.last_received_pos_time[ns].nanoseconds) / 1_000_000.0
                    
                    # Optimize edilmiş zaman hesaplamaları
                    att_age_ms = BIG_TIMEOUT
                    if ns in self.last_received_att_time:
                        att_age_ms = (current_time.nanoseconds - self.last_received_att_time[ns].nanoseconds) / 1_000_000.0
                        
                    sts_age_ms = BIG_TIMEOUT
                    if ns in self.last_received_sts_time:
                        sts_age_ms = (current_time.nanoseconds - self.last_received_sts_time[ns].nanoseconds) / 1_000_000.0
                        
                    gps_age_ms = BIG_TIMEOUT
                    if ns in self.last_received_gps_time:
                        gps_age_ms = (current_time.nanoseconds - self.last_received_gps_time[ns].nanoseconds) / 1_000_000.0
                    
                    battery_age_ms = BIG_TIMEOUT
                    if ns in self.last_received_battery_time:
                        battery_age_ms = (current_time.nanoseconds - self.last_received_battery_time[ns].nanoseconds) / 1_000_000.0
                    
                    gps_info_age_ms = BIG_TIMEOUT
                    if ns in self.last_received_gps_info_time:
                        gps_info_age_ms = (current_time.nanoseconds - self.last_received_gps_info_time[ns].nanoseconds) / 1_000_000.0

                    # Determine timeout conditions
                    position_timed_out = pos_age_ms > position_timeout_ms
                    attitude_timed_out = att_age_ms > attitude_timeout_ms
                    status_timed_out = sts_age_ms > status_timeout_ms
                    # Effective GPS timeout: if PX4 says global position is valid AND this data is old
                    effective_gps_timed_out = drone_data.global_position_valid and (gps_age_ms > global_pos_timeout_ms)
                    battery_timed_out = battery_age_ms > battery_timeout_ms
                    gps_info_timed_out = gps_info_age_ms > gps_info_timeout_ms
                    
                    # Determine if drone is lost (all core sensors timed out)
                    is_lost = position_timed_out and attitude_timed_out and status_timed_out
                    
                    # Even if all data is timed out, keep the drone in the swarm state
                    # But log that it's lost so we're aware
                    if is_lost:
                        # Kaybolan drone'lar için log frekansını azalttık
                        self.get_logger().debug(f"Drone {ns} has no valid data (all timed out), but keeping in swarm state", throttle_duration_sec=10)
                        drone_data.operational_mode = "VERI_YOK"  # Mark drone as having no valid data
                        
                    # Mark global position as invalid if data is too old
                    if effective_gps_timed_out:
                        drone_data.global_position_valid = False
                    
                    # Set new fields based on data or use defaults for timed out data
                    
                    # Set operational mode based on current state
                    if ns in self.operational_modes:
                        drone_data.operational_mode = self.operational_modes.get(ns, "IDLE")
                    else:
                        if not drone_data.armed:
                            drone_data.operational_mode = "IDLE"
                        elif drone_data.flight_mode_str == "TAKEOFF":
                            drone_data.operational_mode = "KALKIS_YAPIYOR"
                        elif drone_data.flight_mode_str == "LAND":
                            drone_data.operational_mode = "INIS_YAPIYOR"
                        elif ns in self.assigned_roles and self.assigned_roles[ns] == "lider":
                            drone_data.operational_mode = "GOREV_UCUSU"
                        elif ns in self.assigned_roles and self.assigned_roles[ns] == "follower":
                            drone_data.operational_mode = "FORMASYONDA"
                        else:
                            drone_data.operational_mode = "GOREV_UCUSU"
                    
                    # Set local planner status
                    drone_data.local_planner_status = self.local_planner_statuses.get(ns, "ENGEL_YOK")
                    
                    # Set assigned role
                    drone_data.assigned_role = self.assigned_roles.get(ns, "")
                    
                    # Handle battery data
                    if battery_timed_out:
                        drone_data.battery_percentage = 0.0  # Unknown/default value
                    
                    # Handle GPS info
                    if gps_info_timed_out:
                        drone_data.gps_fix_type = 0  # No fix
                        drone_data.num_satellites = 0
                        drone_data.estimation_confidence = 0.0
                    
                    # Add to swarm state message
                    swarm_state_msg.drones.append(drone_data)
                    
                    # Calculate swarm centroid if we have valid global positions
                    if drone_data.global_position_valid:
                        sum_lat += drone_data.latitude
                        sum_lon += drone_data.longitude
                        sum_alt += drone_data.altitude
                        sum_x += drone_data.position.x
                        sum_y += drone_data.position.y
                        valid_positions_count += 1
                    
                    # DEBUG modda detaylı log çıktısı oluştur, değilse boş geç
                    if log_swarm_state:
                        log_str = f"[{ns}] [{'OK' if drone_data.global_position_valid else 'NO GPS'}]"
                        if drone_data.global_position_valid:
                            log_str += f" GP: {drone_data.latitude:.6f}, {drone_data.longitude:.6f}, {drone_data.altitude:.1f}m"
                        else:
                            log_str += f" LP: {drone_data.local_x:.1f}, {drone_data.local_y:.1f}, {drone_data.local_z:.1f}m"
                            
                        log_str += f" | {drone_data.flight_mode_str} | {'ARMED' if drone_data.armed else 'DISARMED'}"
                        log_output.append(log_str)
                
                if log_swarm_state:
                    log_output.append(f"Total Drones: {len(swarm_state_msg.drones)}")
                
                # Calculate swarm centroid if we have valid positions
                if valid_positions_count > 0:
                    # Calculate average lat/lon/alt as the centroid
                    avg_lat = sum_lat / valid_positions_count
                    avg_lon = sum_lon / valid_positions_count
                    avg_alt = sum_alt / valid_positions_count
                    
                    # Also calculate local position average for backward compatibility
                    avg_x = sum_x / valid_positions_count
                    avg_y = sum_y / valid_positions_count
                    
                    # Set both the local and global centroids
                    swarm_state_msg.swarm_centroid.x = avg_x  # Average of drone local x positions
                    swarm_state_msg.swarm_centroid.y = avg_y  # Average of drone local y positions
                    swarm_state_msg.swarm_centroid.z = avg_alt  # Average altitude
                    
                    # Set the GPS-based swarm centroid
                    swarm_state_msg.centroid_latitude = avg_lat   # Average latitude
                    swarm_state_msg.centroid_longitude = avg_lon  # Average longitude
                    swarm_state_msg.centroid_altitude = avg_alt   # Average altitude
                    
                    # DEBUG modda sürü merkezi bilgisini logla
                    if log_swarm_state:
                        # Add swarm centroid to the log
                        log_output.append(f"\nSwarm centroid: ({avg_lat:.6f}, {avg_lon:.6f}, {avg_alt:.1f})")
                        # Log the state
                        self.get_logger().debug('\n'.join(log_output))
            
            # Always publish the message if we have drones in the data map
            if swarm_state_msg.drones:
                self.swarm_state_publisher.publish(swarm_state_msg)
            elif self.drone_data_map:
                # This should rarely happen now since we don't skip lost drones
                self.get_logger().warn(
                    f"Drone data map has entries ({list(self.drone_data_map.keys())}), but no drone data was added to the swarm message. "
                    f"This is unexpected since we now keep all drones even when timed out.",
                    throttle_duration_sec=5
                )
            else:  # No configured drones and no data
                self.get_logger().warn("No drones configured and no data to publish.", throttle_duration_sec=5)
        except Exception as e:
            self.get_logger().error(f"Error in publish_swarm_state: {str(e)}")


    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().debug("Configuring SwarmStatePublisherNode...")
        try:
            self._configured_drone_namespaces = self.get_parameter('drone_namespaces').get_parameter_value().string_array_value
            publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

            if not self._configured_drone_namespaces:
                self.get_logger().warn("No drone namespaces configured. Node will not track any drones.")
            
            if publish_frequency <= 0:
                self.get_logger().error(f"Publish frequency must be positive. Got {publish_frequency}")
                return TransitionCallbackReturn.FAILURE 
            publish_period = 1.0 / publish_frequency
            
            now_msg = self.get_clock().now().to_msg()
            for i, ns in enumerate(self._configured_drone_namespaces):
                drone_id_to_assign = i + 1 
                try:
                 
                    #  px4_1, px4_2, px4_3 vs. 
                    parsed_id = int(ns.split('_')[-1])
                    drone_id_to_assign = parsed_id
                except (IndexError, ValueError):
                    self.get_logger().warn(
                        f"Could not parse numeric ID from namespace '{ns}'. Using index {i+1} as drone_id."
                    )
                
                self.drone_data_map[ns] = DroneIndividualState(
                    drone_id=drone_id_to_assign, # ID burada assign ediliyor
                    drone_ns=ns,
                    global_position_valid=False,
                    last_local_pos_timestamp=now_msg,
                    last_attitude_timestamp=now_msg,
                    last_status_timestamp=now_msg,
                    last_global_pos_timestamp=now_msg
                )
                
            qos_sensor_data = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5
            )
            qos_aggregated_state = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
                depth=10
            )

            # subscriber'lar _configured_drone_namespaces listesindeki dronlar için oluşturulur
            for ns in self._configured_drone_namespaces:
                self.get_logger().debug(f"Creating subscriptions for drone: {ns}")
                self.subscribers.append(self.create_subscription(
                    VehicleLocalPosition, f'/{ns}/fmu/out/vehicle_local_position',
                    lambda msg, drone_ns=ns: self.local_position_callback(msg, drone_ns), qos_sensor_data))
                self.subscribers.append(self.create_subscription(
                    VehicleAttitude, f'/{ns}/fmu/out/vehicle_attitude',
                    lambda msg, drone_ns=ns: self.attitude_callback(msg, drone_ns), qos_sensor_data))
                self.subscribers.append(self.create_subscription(
                    Px4VehicleStatus, f'/{ns}/fmu/out/vehicle_status_v1', 
                    lambda msg, drone_ns=ns: self.status_callback(msg, drone_ns), qos_sensor_data))
                self.subscribers.append(self.create_subscription(
                    VehicleGlobalPosition, f'/{ns}/fmu/out/vehicle_global_position',
                    lambda msg, drone_ns=ns: self.global_position_callback(msg, drone_ns), qos_sensor_data))
                # Add subscriptions for battery status and GPS info
                self.subscribers.append(self.create_subscription(
                    BatteryStatus, f'/{ns}/fmu/out/battery_status',
                    lambda msg, drone_ns=ns: self.battery_status_callback(msg, drone_ns), qos_sensor_data))
                # GPS info kullanarak ek konum bilgilerini alıyoruz
                # VehicleGlobalPosition mesajından konum bilgilerini zaten alıyoruz,
                # şimdi ek olarak fix_type ve satelite sayısı gibi bilgileri de kullanacağız
                self.subscribers.append(self.create_subscription(
                    VehicleGlobalPosition, f'/{ns}/fmu/out/vehicle_global_position',
                    lambda msg, drone_ns=ns: self.gps_info_callback(msg, drone_ns), qos_sensor_data))
                
            self.swarm_state_publisher = self.create_lifecycle_publisher(
                SwarmState, '/swarm_state', qos_aggregated_state)
            self.publish_timer = self.create_timer(publish_period, self.publish_swarm_state)
            
            self.get_logger().debug(f"Swarm Data Aggregator configured for: {self._configured_drone_namespaces}")
            return super().on_configure(state) 
        except Exception as e:
            self.get_logger().error(f"Error during configuration: {str(e)}", exc_info=True) 
            return TransitionCallbackReturn.ERROR

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().debug("Activating SwarmStatePublisherNode...")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().debug("Deactivating SwarmStatePublisherNode...")
        return super().on_deactivate(state) 


    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().debug("Cleaning up SwarmStatePublisherNode...")
        if self.publish_timer is not None:
            self.destroy_timer(self.publish_timer) 
            self.publish_timer = None
        
        if self.swarm_state_publisher is not None:
            self.destroy_publisher(self.swarm_state_publisher) 
            self.swarm_state_publisher = None

        self.subscribers.clear() 
            
        self.drone_data_map.clear()
        self.last_received_pos_time.clear()
        self.last_received_att_time.clear()
        self.last_received_sts_time.clear()
        self.last_received_gps_time.clear()
        self.last_received_battery_time.clear()
        self.last_received_gps_info_time.clear()
        self.operational_modes.clear()
        self.local_planner_statuses.clear()
        self.assigned_roles.clear()
        self._configured_drone_namespaces.clear()

        self.get_logger().debug("SwarmStatePublisherNode cleanup complete.")
        return super().on_cleanup(state)
            
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().debug("Shutting down SwarmStatePublisherNode...")
        return super().on_shutdown(state)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmStatePublisherNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().debug("Swarm State Publisher interrupted or executor externally shutdown.")
    finally:
       
        if rclpy.ok():
            executor.shutdown()
        
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
