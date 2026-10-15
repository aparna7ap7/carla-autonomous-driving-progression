#!/usr/bin/env python
"""
CARLA Waypoint Following with Simple Speed Control & Pygame Display (cr_4)

This script demonstrates autonomous navigation along pre-defined waypoints in CARLA. 
It spawns an ego vehicle, defines a sequential path with target velocity targets, 
and uses a 3-state speed controller (`simple_speed_controller`) to modulate throttle 
and braking based on the speed error for each active segment.

Key Features:
- Autonomous Waypoint Tracking: Progresses along waypoints based on distance proximity.
- Longitudinal Speed Control: Adjusts throttle/brake using a rule-based velocity regulator.
- Telemetry & Visualization: Renders real-time camera views via PygameDisplay and 
  plots positional tracking (X-coordinate) alongside velocity profiles over time.
- Lifecycle Management: Properly handles spectator placement and safe cleanup of 
  all spawned actors on execution exit.

Dependencies:
- tools.plotter_x (Plotter class)
- tools.pygame_display (PygameDisplay class)
"""

import carla
import numpy as np 
import time
# Removed typing import

# Import helper classes using the specified format
from tools.plotter_x import Plotter       
from tools.pygame_display import PygameDisplay  

# --- Motion Configuration ---
# Waypoints format: [x_coordinate, y_coordinate, desired_speed_kmh]
WAYPOINTS = [
    [-64.0, 24.5, 40.0], # Start position (speed is for the first segment)
    [40.0,  24.5, 40.0], # Drive towards braking point (maintain 40 km/h)
    [70.0,  24.5, 0.0]   # Stopping point (target speed 0 km/h)
]
# Index of the current target waypoint in the WAYPOINTS list
# Starts at 1 because WAYPOINTS[0] is the spawn point/reference for the first segment.
target_waypoint_id = 1 
# Allowed distance deviation (in meters) to switch to the next waypoint
WAYPOINT_THRESHOLD = 5.0 

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
        # Optionally print that the final destination is reached
        # if target_waypoint_id == len(WAYPOINTS) - 1: print("Final waypoint reached.")
        return

    # Get current target waypoint data
    target_wp_data = WAYPOINTS[target_waypoint_id]
    target_loc = carla.Location(x=target_wp_data[0], y=target_wp_data[1]) # Z not needed for 2D check
    
    # Check the 2D distance to the target point
    distance = vehicle_location.distance_2d(target_loc)
    
    # Switch if close enough
    if distance < WAYPOINT_THRESHOLD:
        reached_wp_id = target_waypoint_id
        target_waypoint_id += 1 # Move to next target index
        print(f"Reached waypoint {reached_wp_id} (dist={distance:.1f}m). New target: {target_waypoint_id}")

# --- Simple Speed Controller ---
def simple_speed_controller(v_desired, v_current):
    """
    Simple 3-state speed controller (bang-bang style).
    Applies full throttle or full brake based on speed comparison.

    Args:
        v_desired: The target speed in km/h.
        v_current: The current speed in km/h.

    Returns:
        A carla.VehicleControl object with throttle/brake set (steer is 0).
    """
    control = carla.VehicleControl()
    control.steer = 0.0 # Steering is not controlled here
    control.throttle = 0.0
    control.brake = 0.0


    if v_desired == 0: # Check for zero speed target
        control.brake = 1.0 # Brake if the target is to stop
        control.throttle = 0.0
    elif v_current < v_desired: 
        control.throttle = 1.0 # Accelerate if driving slower than target
        control.brake = 0.0
    elif v_current > v_desired: 
        control.throttle = 0.0 # Brake if driving faster than target
        control.brake = 1.0
        
    return control

# --- Spectator Setup ---
def set_initial_spectator_view(world):
    """Sets the spectator camera to a predefined fixed position and rotation."""
    spectator = world.get_spectator()
    # Define spectator transform (adjust coordinates as needed for your map/scenario)
    spectator_location = carla.Location(x=63.12, y=29.88, z=5.61)
    spectator_rotation = carla.Rotation(pitch=-4.27, yaw=-170.21, roll=0.00)
    spectator_transform = carla.Transform(spectator_location, spectator_rotation)
    try:
        spectator.set_transform(spectator_transform)
        print(f"Initial spectator position set to: Loc=[{spectator_location.x:.2f}, {spectator_location.y:.2f}, {spectator_location.z:.2f}], Rot=[P:{spectator_rotation.pitch:.2f}, Y:{spectator_rotation.yaw:.2f}, R:{spectator_rotation.roll:.2f}]")
    except Exception as e:
        print(f"Error setting spectator transform: {e}")


def main():
    """Main execution function."""
    # Declare modification of global variables
    global simulation_start_time, target_waypoint_id
    
    client = None
    world = None
    vehicle = None 
    plotter = None 
    pygame_display = None 

    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) # seconds
        print("Connecting to CARLA server...")
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")
        
        # Remove vehicles from previous runs
        remove_previous_vehicle(world)
        
        # --- Vehicle Spawning ---
        map_spawn_points = world.get_map().get_spawn_points()
        if not map_spawn_points:
            print("Error: No spawn points found on the map!")
            return
            
        # Use the starting position from the first waypoint
        start_waypoint = WAYPOINTS[0]
        start_location = carla.Location(x=start_waypoint[0], y=start_waypoint[1], z=0.5) # z=0.5 for safety
        
        # Find the nearest map spawn point to our desired start location (primarily for orientation)
        spawn_point = min(map_spawn_points, key=lambda sp: sp.location.distance(start_location))
        
        # Set the exact location but keep the map's orientation (usually aligned with road)
        spawn_point.location = start_location
        # Optionally force yaw=0 if needed: spawn_point.rotation = carla.Rotation(yaw=0) 
        print(f"Target spawn location: {start_location}, using map rotation: {spawn_point.rotation}")
        
        # Get vehicle blueprint and set role
        vehicle_bp_library = world.get_blueprint_library()
        vehicle_bp = vehicle_bp_library.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'my_car') 
        
        # Try to spawn the vehicle
        print("Attempting to spawn vehicle...")
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
             print(f"Error: Failed to spawn vehicle at {spawn_point.location}")
             return
        print(f"Vehicle {vehicle.type_id} (ID {vehicle.id}) spawned successfully.")
        # --- End Vehicle Spawning ---

        # Set spectator view
        set_initial_spectator_view(world) 

        # --- Initialize Plotter & Pygame ---
        print("Initializing Plotter...")
        plotter = Plotter() # Uses tools.plotter_x
        plotter.init_plot()
        print("Plotter initialized.")
        
        print("Initializing Pygame display...")
        pygame_display = PygameDisplay(world, vehicle) # Uses tools.pygame_display
        print("Pygame display initialized.")
        
        # Record the start time of the main loop
        simulation_start_time = time.time() 
        target_waypoint_id = 1 # Reset target ID at the start

        # --- Main Simulation Loop ---
        print("Simulation running. Following waypoints with simple speed control. Press ESC or close Pygame window to stop.")
        while True:
            current_loop_time = time.time() 

            # Check Pygame events first (for ESC or window close)
            if pygame_display.parse_events():
                print("Quit requested via Pygame window.")
                break 

            # Wait for the next simulation tick from the CARLA server
            world.wait_for_tick() 
            
            # --- Data Collection ---
            current_location = get_location(vehicle) 
            current_speed_kmh = get_speed_kmh(vehicle)
            # Time since simulation start for the plot's X-axis
            current_sim_time_sec = current_loop_time - simulation_start_time 

            # --- Update Visualizations ---
            # Update plots
            if plotter is not None and plotter.is_initialized:
                # Get the desired speed for the *current target* waypoint for plotting
                current_desired_speed_plot = WAYPOINTS[target_waypoint_id][2] if 0 <= target_waypoint_id < len(WAYPOINTS) else 0.0
                # Use try-except for robustness against closed plot window
                try:
                    plotter.update_plot(current_sim_time_sec, current_location.x, current_speed_kmh, current_desired_speed_plot)
                except Exception as plot_update_e:
                    print(f"Error updating plot (likely closed): {plot_update_e}")
                    plotter.cleanup_plot() # Attempt cleanup
                    plotter = None # Stop trying to update plot
            
            # Render Pygame display
            pygame_display.render()

            # --- Waypoint Following Logic ---
            # 1. Update the target waypoint if needed
            update_target_waypoint(current_location)
            
            # 2. Get the desired speed from the current target waypoint
            v_desired = 0.0 # Default speed if index is out of bounds
            if 0 <= target_waypoint_id < len(WAYPOINTS):
                v_desired = WAYPOINTS[target_waypoint_id][2]
            
            # 3. Calculate control using the simple controller
            control = simple_speed_controller(v_desired, current_speed_kmh)
            
            # 4. Apply the calculated control
            vehicle.apply_control(control)
            # --- End Waypoint Following Logic ---

    # Handle user interruption (Ctrl+C)
    except KeyboardInterrupt:
        print("\nScript stopped by user (Ctrl+C).")
    # Handle other potential errors
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}") 
        import traceback
        traceback.print_exc() # Print full traceback for debugging
    
    # --- Resource Cleanup Block ---
    finally:
        print("Starting resource cleanup...")
        
        # Destroy Pygame display first (handles its own camera cleanup)
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
            
        # Destroy the main vehicle
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
    # Run the main function when the script starts
    main()
