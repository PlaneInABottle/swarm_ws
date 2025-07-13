# Aggregated state of the entire swarm
std_msgs/Header header # Timestamp for the aggregated data

swarm_interfaces/DroneIndividualState[] drones # List of states for each drone in the swarm

# Global swarm metrics
geometry_msgs/Point swarm_centroid # Geometric center of the swarm based on local positions

# Global swarm centroid based on GPS
float64 centroid_latitude  # Centroid latitude in degrees (WGS84)
float64 centroid_longitude # Centroid longitude in degrees (WGS84)
float32 centroid_altitude  # Centroid altitude in meters (AMSL)