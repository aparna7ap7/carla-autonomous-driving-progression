#!/usr/bin/env python
"""
CARLA Vehicle Control and Smooth Spectator Following (cr_2)

This script connects to a running CARLA simulator, spawns a vehicle, and 
applies a continuous forward throttle to move it in a straight line.

It introduces a dynamic camera tracking system that smoothly updates 
the spectator's transform in real time. The camera follows the vehicle from a 
fixed offset (positioned behind and slightly above, angled downward) by 
continuously reading the vehicle's current position and orientation in the main loop.
"""

import carla
import time

# Constants for relative spectator camera positioning
DISTANCE_BACK = 15.0 # Meters behind the vehicle
HEIGHT_UP = 6.0      # Meters above the vehicle's base
PITCH_DOWN = -20.0   # Degrees to tilt the camera down

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


def get_transform_for_spectator(vehicle_transform):
    """
    Calculates the desired spectator transform based on the vehicle's transform.
    The camera is positioned behind, above, and looking down at the vehicle,
    relative to the vehicle's current orientation.

    Args:
        vehicle_transform: The current carla.Transform of the vehicle.

    Returns:
        A carla.Transform representing the desired spectator camera position and orientation.
    """
    forward_vector = vehicle_transform.get_forward_vector()
    
    # Calculate offset: -DISTANCE_BACK meters back, HEIGHT_UP meters up
    # Manually calculate x and y components for the backward offset
    offset = carla.Location(
        x=-DISTANCE_BACK * forward_vector.x, 
        y=-DISTANCE_BACK * forward_vector.y, 
        z=HEIGHT_UP # Direct z offset
    )
    
    # Final spectator location
    location = vehicle_transform.location + offset
    
    # Spectator rotation: use the vehicle's yaw and a fixed pitch
    rotation = carla.Rotation(pitch=PITCH_DOWN, yaw=vehicle_transform.rotation.yaw)
    
    return carla.Transform(location, rotation)


def main():
    """Main execution function."""
    client = None
    world = None
    vehicle = None # Initialize for the finally block

    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) # seconds
        print("Connecting to CARLA server...")
        
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")
        
        # Remove any old vehicle
        remove_previous_vehicle(world)

        # Spawn the vehicle
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Error: No spawn points found on the map!")
            return
        spawn_point = spawn_points[0] # Use the first available spawn point
        print(f"Selected spawn point: {spawn_point.location}")
        
        vehicle_bp_library = world.get_blueprint_library()
        vehicle_bp = vehicle_bp_library.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'my_car') # Assign role
        
        print("Attempting to spawn vehicle...")
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        if vehicle is None:
            print(f"Error: Failed to spawn vehicle at {spawn_point.location}")
            return
            
        print(f"Vehicle {vehicle.type_id} (ID {vehicle.id}) spawned successfully.")

        # Wait a moment for the vehicle to settle
        world.wait_for_tick()
        time.sleep(0.5)

        # Set initial spectator position based on the spawned vehicle's transform
        spectator = world.get_spectator() 
        try:
            spectator.set_transform(get_transform_for_spectator(vehicle.get_transform())) 
            print("Initial spectator position set.")
        except Exception as e:
            print(f"Error setting initial spectator transform: {e}")


        # --- Main Simulation Loop ---
        print("Simulation running. Applying constant throttle. Spectator following. Press Ctrl+C to stop.")
        frame = 0
        while True:
            world.wait_for_tick() # Synchronize with the simulator tick
            frame += 1
            
            # Get current vehicle transform
            current_transform = vehicle.get_transform()
            
            # Apply constant control to the vehicle (move forward)
            control = carla.VehicleControl(throttle=0.8, steer=0.0, brake=0.0) 
            vehicle.apply_control(control)

            # Update spectator position smoothly every tick
            try:
                 new_spectator_transform = get_transform_for_spectator(current_transform)
                 spectator.set_transform(new_spectator_transform)
                 # Optional: Print spectator position periodically for debugging
                 # if frame % 100 == 0: 
                 #    loc = new_spectator_transform.location
                 #    rot = new_spectator_transform.rotation
                 #    print(f"Frame {frame}: Spectator at Loc=[{loc.x:.1f},{loc.y:.1f},{loc.z:.1f}], Rot=[P:{rot.pitch:.1f},Y:{rot.yaw:.1f}]")
            except Exception as e:
                 print(f"Error updating spectator transform: {e}")
                 # Decide if simulation should continue or stop on spectator error

    except KeyboardInterrupt:
        print("\nScript stopped by user (Ctrl+C).")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Clean up resources
        print("Starting resource cleanup...")
        if vehicle is not None and vehicle.is_alive:
             # It's good practice to disable physics before destroying
             # vehicle.set_simulate_physics(False) # Might cause issues if physics was already off
             print(f"Destroying vehicle: {vehicle.type_id} (ID {vehicle.id})")
             if vehicle.destroy():
                 print("Vehicle destroyed successfully.")
             else:
                 print("Vehicle destroy() returned False.")
        else:
            print("Vehicle was None or not alive, no destruction needed.")
            
        print("Simulation finished.")


if __name__ == '__main__':
    main()
