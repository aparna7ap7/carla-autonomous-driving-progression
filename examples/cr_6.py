#!/usr/bin/env python
"""
CARLA Waypoint Following with PID Speed, Stanley Steering & Pygame Display (cr_6)

This script demonstrates full autonomous path-tracking control in CARLA, combining longitudinal 
PID velocity regulation with lateral Stanley steering control along a complex waypoint trajectory.

Key Features:
- Dual-Loop Control Architecture: Uses a PID controller (`pid_controller.py`) for precise 
  longitudinal speed regulation and the Stanley geometric method (`calculate_stanley_steering_darpa`) 
  for robust lateral path tracking.
- Advanced Waypoint Management: Dynamically sequences through target waypoints based on proximity thresholds.
- Comprehensive Visualization: Renders live camera streams via PygameDisplay alongside 
  multi-panel Matplotlib telemetry tracking 2D spatial trajectories (X vs. Y) and speed profiles over time.
- Clean Resource Handling: Manages spectator views and ensures proper actor cleanup upon execution termination.

Dependencies:
- tools.plotter_path (Plotter class for trajectory and speed visualization)
- tools.pygame_display (PygameDisplay class)
- tools.pid_controller (PIDController class)
"""

import carla
import numpy as np 
import time
import math
# Removed typing import

# Import helper classes using the specified format
from tools.plotter_path import Plotter       
from tools.pygame_display import PygameDisplay 
from tools.pid_controller import PIDController 

# --- Motion Configuration ---
# Waypoints format: [x_coordinate, y_coordinate, desired_speed_kmh]
WAYPOINTS = [
    # Start position (spawn point, speed is for the first segment)
    [-64.0, 24.5, 40.0], 
    # 1. Drive towards the braking/turning point
    [0.0,   24.5, 40.0],   # Maintain 40 km/h
    # 2. Start braking and turning/lane change
    [27.0,  28.0, 20.0],   # Reduce speed to 20 km/h
    # 3. Slow driving after lane change
    [37.0,  32.0, 15.0],   # Drive slowly (15 km/h)
    # 4. Continue slow driving
    [40.0,  40.0, 15.0],   # Maintain 15 km/h
    # 5. Point before crosswalk
    [40.7, 53.4, 15.0],   # Maintain 15 km/h
    # 6. Point representing stop or slow down (adjust speed as needed)
    [40.7, 53.4, 15.0],   # Maintain 15 km/h (or set to 0 for a stop)
    # 7. Start moving and turning after stop/slowdown
    [38.8, 61.6, 20.0],   # Accelerate to 20 km/h for the turn
    # 8. Continue turn
    [34.6, 63.0, 20.0],   # Maintain 20 km/h
    # 9. Straighten out
    [25.0, 66.0, 30.0],   # Accelerate to 30 km/h
    # 10. Point before braking for final stop
    [-9.8, 66.1, 30.0],   # Maintain 30 km/h up to here
    # 11. Approach final stop
    [-23.0, 66.0, 20.0],  # Reduce speed to 20 km/h
    # 12. Final stop
    [-26.0, 66.0, 0.0]    # Target speed 0
]

# Index of the current target waypoint in the WAYPOINTS list
target_waypoint_id = 1 
# Allowed distance deviation (in meters) to switch to the next waypoint
WAYPOINT_THRESHOLD = 3.0 # Can be tuned for precision vs. smoothness

# Simulation start time (for plotting time axis)
simulation_start_time = 0.0 

# --- Vehicle Data Functions ---
def get_location(vehicle):
   """Returns the current location of the vehicle."""
   return vehicle.get_location()

def get_speed_kmh(vehicle):
   """Returns the current speed of the vehicle in km/h."""
   velocity_vector = vehicle.get_velocity()
   speed_meters_per_second = np.linalg.norm([velocity_vector.x, velocity_vector.y, velocity_vector.z]) 
   return 3.6 * speed_meters_per_second 

def get_speed_mps(vehicle):
   """Returns the current speed of the vehicle in m/s."""
   velocity_vector = vehicle.get_velocity()
   return np.linalg.norm([velocity_vector.x, velocity_vector.y, velocity_vector.z])

# --- Simulation Management Functions ---
def remove_previous_vehicle(world):
    """Finds and removes all vehicles with the role 'my_car'."""
    print("Searching for previous 'my_car' vehicles...")
    actors = world.get_actors().filter('vehicle.*')
    count = 0
    for actor in actors:
        if actor.attributes.get('role_name') == 'my_car':
            print(f"  - Removing previous vehicle: {actor.type_id} (ID {actor.id})")
            if actor.destroy():
                 count += 1
            else:
                 print(f"  - Failed to remove vehicle {actor.id}")
    print(f"Removed {count} previous vehicles.")

def update_target_waypoint(vehicle_location):
    """
    Checks if the current target waypoint is reached and switches to the next one.
    Updates the global `target_waypoint_id`.
    """
    global target_waypoint_id # Allow modification of the global variable
    
    # If this is the last actual target point (index len-1), do not switch further.
    if target_waypoint_id >= len(WAYPOINTS) - 1:
        return

    # Get current target waypoint data
    target_wp_data = WAYPOINTS[target_waypoint_id]
    target_loc = carla.Location(x=target_wp_data[0], y=target_wp_data[1]) 
    
    distance = vehicle_location.distance_2d(target_loc)
    
    # Switch if close enough
    if distance < WAYPOINT_THRESHOLD:
        reached_wp_id = target_waypoint_id
        reached_wp_coords = (target_wp_data[0], target_wp_data[1]) 
        
        target_waypoint_id += 1 # Move to next target index
        
        # Log the switch
        log_message = f"Reached waypoint {reached_wp_id} (X={reached_wp_coords[0]:.1f}, Y={reached_wp_coords[1]:.1f}, dist={distance:.1f}m). "
        if target_waypoint_id < len(WAYPOINTS):
            new_target_wp_data = WAYPOINTS[target_waypoint_id]
            new_target_coords = (new_target_wp_data[0], new_target_wp_data[1])
            log_message += f"New target: {target_waypoint_id} (X={new_target_coords[0]:.1f}, Y={new_target_coords[1]:.1f})"
        else:
             log_message += "Final waypoint index reached."
        print(log_message)


# --- Spectator Setup ---
def set_initial_spectator_view(world):
    """Sets the spectator camera to a predefined fixed position and rotation."""
    spectator = world.get_spectator()
    # Define spectator transform (adjust coordinates as needed)
    spectator_location = carla.Location(x=63.12, y=29.88, z=5.61)
    spectator_rotation = carla.Rotation(pitch=-4.27, yaw=-170.21, roll=0.00)
    spectator_transform = carla.Transform(spectator_location, spectator_rotation)
    try:
        spectator.set_transform(spectator_transform)
        print(f"Initial spectator position set to: Loc=[{spectator_location.x:.2f}, {spectator_location.y:.2f}, {spectator_location.z:.2f}], Rot=[P:{spectator_rotation.pitch:.2f}, Y:{spectator_rotation.yaw:.2f}, R:{spectator_rotation.roll:.2f}]")
    except Exception as e:
        print(f"Error setting spectator transform: {e}")


# --- Controller Constants and Functions ---
MIN_SPEED_STANLEY = 1e-4 # Minimum speed (m/s) for Stanley arctan calculation to avoid division by zero
STANLEY_K = 0.5          # Stanley controller gain for cross-track error component
MAX_STEER_DEG = 70.0     # Maximum steering angle in degrees (for clamping and normalization)
MAX_STEER_RAD = math.radians(MAX_STEER_DEG) # Maximum steering angle in radians


def calculate_stanley_steering_darpa(vehicle, waypoints, current_target_id):
    """Calculates the steering angle using the Stanley method (DARPA version - restored logic)."""
    
    k = STANLEY_K # Use the constant

    # Check if there is a previous waypoint
    if current_target_id < 1:
        return 0.0

    # Vehicle data
    vehicle_location = vehicle.get_location()
    vehicle_transform = vehicle.get_transform()
    vehicle_yaw_rad = math.radians(vehicle_transform.rotation.yaw)
    vehicle_speed_mps = get_speed_mps(vehicle)

    # Previous and target waypoints
    wp_prev = waypoints[current_target_id - 1]
    wp_target = waypoints[current_target_id]

    # Path segment angle
    yaw_path = np.arctan2(wp_target[1] - wp_prev[1], wp_target[0] - wp_prev[0])

    # Heading error
    yaw_diff = yaw_path - vehicle_yaw_rad
    # Normalize [-pi, pi]
    if yaw_diff > np.pi:
        yaw_diff -= 2 * np.pi
    if yaw_diff < -np.pi:
        yaw_diff += 2 * np.pi

    # Cross-track error (CTE) calculation
    # Using the line equation method from the previous version
    diff_x = wp_target[0] - wp_prev[0]
    diff_y = wp_target[1] - wp_prev[1]
    if abs(diff_x) < 1e-6: 
        crosstrack_error = vehicle_location.x - wp_prev[0]
    else:
        slope = diff_y / diff_x
        a = -slope
        b = 1.0
        c = (slope * wp_prev[0]) - wp_prev[1]
        crosstrack_error = (a * vehicle_location.x + b * vehicle_location.y + c) / np.sqrt(a ** 2 + b ** 2)

    # Determine CTE sign (as in the previous version)
    yaw_cross_track = np.arctan2(vehicle_location.y - wp_prev[1], vehicle_location.x - wp_prev[0])
    yaw_path2ct = yaw_path - yaw_cross_track
    # Normalize [-pi, pi]
    if yaw_path2ct > np.pi:
        yaw_path2ct -= 2 * np.pi
    if yaw_path2ct < -np.pi:
        yaw_path2ct += 2 * np.pi
        
    # Apply sign to CTE
    if yaw_path2ct > 0:
        crosstrack_error = abs(crosstrack_error)
    else:
        crosstrack_error = -abs(crosstrack_error)

    # Steering component from CTE
    safe_speed = max(vehicle_speed_mps, MIN_SPEED_STANLEY) 
    yaw_diff_crosstrack = np.arctan(k * crosstrack_error / safe_speed)

    # Expected final steering angle (in radians)
    steer_expect = yaw_diff + yaw_diff_crosstrack
    # Normalize [-pi, pi]
    if steer_expect > np.pi:
        steer_expect -= 2 * np.pi
    if steer_expect < -np.pi:
        steer_expect += 2 * np.pi
        
    # Limit steering angle in radians (using MAX_STEER_DEG for conversion)
    max_steer_rad = math.radians(MAX_STEER_DEG) # Convert max degree to radians
    steer_expect = min(max_steer_rad, steer_expect)
    steer_expect = max(-max_steer_rad, steer_expect)

    # Convert to CARLA control signal [-1, 1] by normalizing with max_steer_rad
    steer_carla = steer_expect / max_steer_rad
    steer_carla = np.clip(steer_carla, -1.0, 1.0) # Ensure it's within bounds

    return steer_carla


def main():
    """Main execution function."""
    global simulation_start_time, target_waypoint_id # Allow modification
    
    client = None
    world = None
    vehicle = None 
    plotter = None 
    pygame_display = None 
    pid_controller = None 

    try:
        # Connect to CARLA
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) 
        print("Connecting to CARLA server...")
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")
        
        # Cleanup previous actors
        remove_previous_vehicle(world)
        
        # --- Vehicle Spawning ---
        map_spawn_points = world.get_map().get_spawn_points()
        if not map_spawn_points:
            print("Error: No spawn points found on the map!")
            return
            
        start_waypoint = WAYPOINTS[0]
        start_location = carla.Location(x=start_waypoint[0], y=start_waypoint[1], z=0.5) # Use z=0.5 for safety
        
        spawn_point = min(map_spawn_points, key=lambda sp: sp.location.distance(start_location))
        spawn_point.location = start_location
        spawn_point.rotation = carla.Rotation(yaw=0) # Force initial yaw to 0 for consistency with waypoints
        print(f"Target spawn location: {start_location}, Using rotation: {spawn_point.rotation}")
        
        vehicle_bp_library = world.get_blueprint_library()
        vehicle_bp = vehicle_bp_library.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'my_car') 
        
        print("Attempting to spawn vehicle...")
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
             print(f"Error: Failed to spawn vehicle at {spawn_point.location}")
             return
        print(f"Vehicle {vehicle.type_id} (ID {vehicle.id}) spawned successfully.")
        # --- End Vehicle Spawning ---

        # Set spectator view
        set_initial_spectator_view(world) 

        # --- Initialize Plotter, Pygame, and PID Controller ---
        print("Initializing Plotter...")
        plotter = Plotter(waypoints=WAYPOINTS) # Pass waypoints to plotter_path
        plotter.init_plot() 
        print("Plotter initialized.")
        
        print("Initializing Pygame display...")
        pygame_display = PygameDisplay(world, vehicle) 
        print("Pygame display initialized.")
        
        print("Initializing PID controller...")
        pid_controller = PIDController() 
        print("PID controller initialized.")
        
        # Record simulation start time
        simulation_start_time = time.time() 
        target_waypoint_id = 1 # Reset target ID

        # --- Main Simulation Loop ---
        print("Simulation running. Following waypoints with PID (Speed) and Stanley (Steering). Press ESC or close window to stop.")
        while True:
            current_loop_time = time.time()

            # Check Pygame events first (for ESC or window close)
            if pygame_display.parse_events():
                print("Quit requested via Pygame window.")
                break 

            # Wait for the next simulation tick
            world.wait_for_tick() 
            
            # --- Data Collection ---
            current_transform = vehicle.get_transform() 
            current_location = current_transform.location 
            current_speed_kmh = get_speed_kmh(vehicle) 
            current_sim_time_sec = current_loop_time - simulation_start_time 

            # --- Update Visualizations ---
            # Update Plotter (Path Plotter)
            if plotter is not None and plotter.is_initialized:
                target_speed_for_plot = WAYPOINTS[target_waypoint_id][2] if 0 <= target_waypoint_id < len(WAYPOINTS) else 0.0
                try:
                    # plotter_path expects time, x, y, current_speed, desired_speed
                    plotter.update_plot(current_sim_time_sec, 
                                        current_location.x, 
                                        current_location.y, 
                                        current_speed_kmh, 
                                        target_speed_for_plot) 
                except Exception as plot_update_e:
                    print(f"Error updating plot (likely closed): {plot_update_e}")
                    plotter.cleanup_plot()
                    plotter = None 
            
            # Render Pygame display
            pygame_display.render()

            # --- Control Logic ---
            # 1. Update target waypoint based on proximity
            update_target_waypoint(current_location)
            
            # 2. Get desired speed for the current segment
            v_desired = 0.0 
            if 0 <= target_waypoint_id < len(WAYPOINTS):
                v_desired = WAYPOINTS[target_waypoint_id][2]
            
            # 3. Calculate longitudinal control (throttle/brake) using PID
            longitudinal_control = pid_controller.calculate_control(v_desired, current_speed_kmh)
            
            # 4. Calculate lateral control (steering) using Stanley
            steering_angle = calculate_stanley_steering_darpa(vehicle, WAYPOINTS, target_waypoint_id) 
            
            # 5. Combine control commands
            control = carla.VehicleControl()
            control.throttle = longitudinal_control.throttle
            control.brake = longitudinal_control.brake
            control.steer = steering_angle 
            control.hand_brake = False
            control.manual_gear_shift = False
            
            # 6. Apply final control to the ego vehicle
            vehicle.apply_control(control)
            # --- End Control Logic ---

    except KeyboardInterrupt:
        print("\nScript stopped by user (Ctrl+C).")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}") 
        import traceback
        traceback.print_exc()
    
    # --- Resource Cleanup Block ---
    finally:
        print("Starting resource cleanup...")
        
        # Destroy Pygame display first 
        if pygame_display is not None:
            print("Destroying Pygame display...")
            pygame_display.destroy()
            print("Pygame display destroyed.")

        # Cleanup plotter
        if plotter is not None:
             if plotter.is_initialized:
                 print("Cleaning up plotter...")
                 plotter.cleanup_plot()
                 print("Plotter cleaned up.")
            
        # Destroy vehicle
        if vehicle is not None and vehicle.is_alive:
             print(f"Destroying vehicle: {vehicle.type_id} (ID {vehicle.id})")
             # vehicle.set_simulate_physics(False) # Optional
             if vehicle.destroy():
                 print("Vehicle destroyed successfully.")
             else:
                 print("Vehicle destroy() returned False.")
        else:
             print("Vehicle was None or not alive, no destruction needed.")
        
        print("Simulation finished.")


if __name__ == '__main__':
    main()
