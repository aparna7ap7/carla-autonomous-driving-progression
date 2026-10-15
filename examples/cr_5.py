#!/usr/bin/env python
"""
CARLA Waypoint Following with PID Speed Control & Pygame Display (cr_5)

This script implements longitudinal vehicle control in CARLA using a proportional–integral–derivative
(PID) controller to precisely regulate speed along a series of sequential waypoints.

Key Features:
- Closed-Loop Speed Control: Utilizes a dedicated PID controller (`pid_controller.py`) to 
  dynamically calculate continuous throttle and braking commands based on velocity error.
- Sequential Waypoint Navigation: Tracks active path segments and updates target waypoints 
  automatically upon reaching spatial proximity thresholds.
- Multi-Window Diagnostics: Captures live vehicle-mounted camera feed using PygameDisplay 
  and plots high-precision speed and spatial telemetry (X-coordinate) over time via Matplotlib.
- Clean Resource Handling: Ensures proper actor destruction, window cleanup, and spectator 
  positioning upon simulation exit.

Dependencies:
- tools/plotter_x.py (Plotter class)
- tools/pygame_display.py (PygameDisplay class)
- tools/pid_controller.py (PIDController class)
"""

import carla
import numpy as np 
import time
import pygame # Import Pygame

# Import helper classes from 'tools' directory
from tools.plotter_x import Plotter       
from tools.pygame_display import PygameDisplay # Use PygameDisplay
from tools.pid_controller import PIDController 

# --- Motion Configuration ---
# Waypoints format: [x_coordinate, y_coordinate, desired_speed_kmh]
WAYPOINTS = [
    [-64, 24.5, 40], # Start position (speed is for the first segment)
    [40,  24.5, 40], # Drive towards the braking point (maintain 40 km/h)
    [70,  24.5, 0]   # Stopping point (target speed 0 km/h)
]

# Index of the current target waypoint in the WAYPOINTS list
target_waypoint_id = 1 
# Allowed distance deviation (in meters) to switch to the next waypoint
WAYPOINT_THRESHOLD = 5.0 

# Simulation start time (for plotting time axis)
simulation_start_time = 0 

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
    """Checks if the current target waypoint is reached and switches to the next one."""
    global target_waypoint_id
    
    if target_waypoint_id >= len(WAYPOINTS) - 1:
        return

    target_wp_data = WAYPOINTS[target_waypoint_id]
    target_loc = carla.Location(x=target_wp_data[0], y=target_wp_data[1]) 
    
    distance = vehicle_location.distance_2d(target_loc)
    
    if distance < WAYPOINT_THRESHOLD:
        reached_wp_id = target_waypoint_id
        target_waypoint_id += 1 
        print(f"Reached waypoint {reached_wp_id}. New target: {target_waypoint_id}")


# --- Spectator Setup ---
def set_initial_spectator_view(world):
    """Sets the spectator camera to a predefined fixed position and rotation."""
    spectator = world.get_spectator()
    # Define spectator transform using the desired fixed values
    spectator_location = carla.Location(x=63.12, y=29.88, z=5.61)
    spectator_rotation = carla.Rotation(pitch=-4.27, yaw=-170.21, roll=0.00)
    spectator_transform = carla.Transform(spectator_location, spectator_rotation)
    try:
        spectator.set_transform(spectator_transform)
        print(f"Initial spectator position set to: Loc=[{spectator_location.x:.2f}, {spectator_location.y:.2f}, {spectator_location.z:.2f}], Rot=[P:{spectator_rotation.pitch:.2f}, Y:{spectator_rotation.yaw:.2f}, R:{spectator_rotation.roll:.2f}]")
    except Exception as e:
        print(f"Error setting spectator transform: {e}")


def main():
    global simulation_start_time, target_waypoint_id
    
    client = None
    world = None
    vehicle = None 
    plotter = None 
    pygame_display = None # Changed variable name
    pid_controller = None 

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) 
        world = client.get_world()
        
        remove_previous_vehicle(world)
        
        map_spawn_points = world.get_map().get_spawn_points()
        if not map_spawn_points:
            print("Error: No spawn points found on the map!")
            return
            
        start_waypoint = WAYPOINTS[0]
        start_location = carla.Location(x=start_waypoint[0], y=start_waypoint[1], z=0.1) 
        
        spawn_point = min(map_spawn_points, key=lambda sp: sp.location.distance(start_location))
        
        spawn_point.location = start_location
        spawn_point.rotation = carla.Rotation(yaw=0) 
        print(f"Target spawn location: {start_location}, Using rotation: {spawn_point.rotation}")
        
        vehicle_bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'my_car') 
        
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
             print(f"Error: Failed to spawn vehicle at {spawn_point.location}")
             return
        print(f"Vehicle {vehicle.type_id} (ID {vehicle.id}) spawned at start position.")
        

        set_initial_spectator_view(world) 

        # --- Initialize Plotter & Pygame ---
        print("Initializing Plotter & Pygame ...")
        plotter = Plotter()
        plotter.init_plot()

        pygame_display = PygameDisplay(world, vehicle)
        
        # Initialize PID Controller
        pid_controller = PIDController() 
        print("PID controller initialized.")
        
        simulation_start_time = time.time() 
        target_waypoint_id = 1 # Reset target ID

        # --- Main Simulation Loop ---
        print("Simulation running. Following waypoints with PID speed control. Press ESC or close Pygame window to stop.")
        while True:
            # Check Pygame events first (for ESC or window close)
            if pygame_display.parse_events():
                break 

            world.wait_for_tick() 
            
            # --- Data Collection ---
            current_transform = vehicle.get_transform() 
            current_location = current_transform.location 
            current_speed_kmh = get_speed_kmh(vehicle) 
            current_sim_time_sec = time.time() - simulation_start_time 

            # --- Update Visualizations ---
            # Update Plotter
            if plotter and plotter.is_initialized:
                target_speed_for_plot = 0
                if 0 <= target_waypoint_id < len(WAYPOINTS):
                    target_speed_for_plot = WAYPOINTS[target_waypoint_id][2]
                
                try:    
                    # Pass time, x, current speed, desired speed to plotter_3
                    plotter.update_plot(current_sim_time_sec, current_location.x, current_speed_kmh, target_speed_for_plot) 
                except Exception as plot_update_e:
                     print(f"Error updating plot (likely closed): {plot_update_e}")
                     plotter = None 
            
            # Render Pygame display
            pygame_display.render() # Update the Pygame window content

            # --- Waypoint Following Logic ---
            update_target_waypoint(current_location)
            
            v_desired = 0 
            if 0 <= target_waypoint_id < len(WAYPOINTS):
                v_desired = WAYPOINTS[target_waypoint_id][2]
            
            longitudinal_control = pid_controller.calculate_control(v_desired, current_speed_kmh)
            
            control = carla.VehicleControl()
            control.throttle = longitudinal_control.throttle
            control.brake = longitudinal_control.brake
            control.steer = 0.0 # No steering
            control.hand_brake = False
            control.manual_gear_shift = False

            vehicle.apply_control(control)
            # --- End Waypoint Following Logic ---

    except KeyboardInterrupt:
        print("\nScript stopped by user.")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}") 
        import traceback
        traceback.print_exc() 
    
    # --- Resource Cleanup Block ---
    finally:
        print("Starting resource cleanup...")
        
        # Destroy Pygame display first
        if pygame_display is not None:
            pygame_display.destroy()

        # Close the plot window
        if plotter:
            plotter.cleanup_plot()
            
        # Destroy the vehicle
        if vehicle is not None and vehicle.is_alive:
             try:
                 vehicle.set_simulate_physics(False) 
                 vehicle.destroy()
                 print(f"Vehicle {vehicle.id} destroyed.")
             except Exception as e:
                 print(f"Error destroying vehicle {vehicle.id}: {e}")
        
        print("Simulation finished.")


if __name__ == '__main__':
    main()
