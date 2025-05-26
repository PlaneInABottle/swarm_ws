import numpy as np


def calculate_simple_avoidance_velocity(
    current_position_ned: np.ndarray,
    other_drones_ned_positions: list[np.ndarray],
    base_desired_velocity_ned: np.ndarray,
    safety_radius: float,
    repulsion_strength: float,
    max_overall_speed: float,
) -> np.ndarray:
    """
    Calculates an adjusted velocity to avoid other drones based on simple repulsion.
    Focuses primarily on horizontal (x, y) avoidance but uses 3D vectors.

    Args:
        current_position_ned: Own drone's current [x, y, z] position (NED).
        other_drones_ned_positions: List of other drones' [x, y, z] positions (NED).
        base_desired_velocity_ned: Drone's intended velocity [vx, vy, vz] (NED) if no obstacles.
        safety_radius: If another drone is within this radius, repulsion occurs.
        repulsion_strength: Scaling factor for the repulsive velocity.
        max_overall_speed: The magnitude of the returned velocity will be capped at this.

    Returns:
        np.ndarray: Adjusted desired velocity vector [vx, vy, vz] (NED).
    """
    total_repulsive_velocity_ned = np.array([0.0, 0.0, 0.0])

    for other_drone_position_ned in other_drones_ned_positions:
        vector_to_other = other_drone_position_ned - current_position_ned
        distance_to_other = np.linalg.norm(vector_to_other)

        # Consider avoidance primarily in the horizontal plane for simplicity,
        # but use 3D distance. You could make this purely 2D if z-separation is handled elsewhere.
        # For this example, we use 3D distance but might primarily apply horizontal repulsion.

        if 0 < distance_to_other < safety_radius:
            # Too close, calculate repulsion
            # Repulsion is stronger when closer
            # Direction is directly away from the other drone

            # Normalized vector pointing away from the other drone
            direction_away = -vector_to_other / distance_to_other

            # Strength of repulsion (linear, stronger when closer)
            # (safety_radius - distance_to_other) gives a value from ~0 to safety_radius
            # Divide by safety_radius to normalize this "penetration depth" from 0 to 1
            repulsion_magnitude = (
                repulsion_strength * (safety_radius - distance_to_other) / safety_radius
            )

            single_repulsive_velocity = repulsion_magnitude * direction_away

            # For this simple example, let's say we only apply horizontal repulsion
            # and let the base_desired_vel_ned handle the z component unless collision is imminent in Z.
            # You could make this fully 3D: total_repulsive_vel_ned += single_repulsive_vel
            total_repulsive_velocity_ned[0] += single_repulsive_velocity[
                0
            ]  # Add x component
            total_repulsive_velocity_ned[1] += single_repulsive_velocity[
                1
            ]  # Add y component
            # total_repulsive_vel_ned[2] += single_repulsive_vel[2] # Optionally add z component

    # Combine base desired velocity with repulsive velocity
    adjusted_velocity_ned = base_desired_velocity_ned + total_repulsive_velocity_ned

    # Cap final velocity to max_overall_speed
    speed = np.linalg.norm(adjusted_velocity_ned)
    if speed > max_overall_speed and speed > 0:
        adjusted_velocity_ned = (adjusted_velocity_ned / speed) * max_overall_speed

    return adjusted_velocity_ned
