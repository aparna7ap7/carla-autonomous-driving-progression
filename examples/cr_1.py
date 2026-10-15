#!/usr/bin/env python
"""
CARLA Basic Vehicle Spawn and Spectator Setup (cr_1)

This script connects to a running CARLA simulator instance, scans for 
and removes any pre-existing vehicle actors with the role 'my_car', and 
spawns a new Tesla Model 3 at a valid spawn point in Town10HD_Opt. 

Once spawned, it repositions the spectator camera directly behind the vehicle 
for real-time visual tracking and maintains the simulation loop in a 
stationary state until terminated (Ctrl+C).
"""
import carla
import time # Added for world.wait_for_tick() usage example

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


def set_spectator_behind_vehicle(world, vehicle):
    """Positions the spectator camera behind the specified vehicle."""
    spectator = world.get_spectator()
    transform = vehicle.get_transform()
    # Get the forward vector to calculate the offset relative to the vehicle's orientation
    forward_vector = transform.get_forward_vector()

    # Calculate offset: -15 meters back, 6 meters up
    offset = carla.Location(x=-15 * forward_vector.x, y=-15 * forward_vector.y, z=6)
    # Calculate the new spectator transform
    spectator_transform = carla.Transform(
        transform.location + offset,
        # Set rotation: slightly looking down (-20 pitch), same yaw as vehicle
        carla.Rotation(pitch=-20, yaw=transform.rotation.yaw, roll=transform.rotation.roll)
    )
    # Wrap the set_transform call in a try-except block for robustness
    try:
        spectator.set_transform(spectator_transform)
        print("Spectator camera positioned behind the vehicle.")
    except Exception as e:
        print(f"Error setting spectator transform: {e}")


def main():
    """Main execution function."""
    client = None
    world = None
    vehicle = None
    
    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0) # seconds
        print("Connecting to CARLA server...")
        
        world = client.get_world()
        print(f"Connected to world: {world.get_map().name}")

        # Remove any old vehicle left from previous runs
        remove_previous_vehicle(world)

        # Get a spawn point (using the first one available on the map for simplicity)
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("Error: No spawn points found on the map!")
            return
        spawn_point = spawn_points[0]

        # Get the vehicle blueprint and set its role
        vehicle_bp_library = world.get_blueprint_library()
        vehicle_bp = vehicle_bp_library.filter('vehicle.tesla.model3')[0]
        vehicle_bp.set_attribute('role_name', 'my_car') # Assign role for easy identification/cleanup
        
        # Try to spawn the vehicle
        print("Attempting to spawn vehicle...")
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
        
        if vehicle is None:
            # Attempt failed, maybe the spawn point is occupied
            print(f"Error: Failed to spawn vehicle at {spawn_point.location}. Trying world.spawn_actor...")
            # Optional: Try forcing spawn (might cause collisions)
            # vehicle = world.spawn_actor(vehicle_bp, spawn_point) 
            # if vehicle is None:
            #     print("Error: world.spawn_actor also failed.")
            #     return
            return # Exit if spawning fails

        print(f"Vehicle {vehicle.type_id} (ID {vehicle.id}) spawned successfully.")

        # Wait a moment for the vehicle to settle before positioning camera
        world.wait_for_tick()
        time.sleep(0.5) 

        # Position the spectator camera
        set_spectator_behind_vehicle(world, vehicle)

        # Keep the script running and the simulation ticking
        print("Simulation running. Vehicle is stationary. Press Ctrl+C to stop.")
        while True:
            world.wait_for_tick() # Wait for the simulator tick to advance time
            time.sleep(0.1) # Small delay to prevent busy-waiting

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
            print(f"Destroying vehicle: {vehicle.type_id} (ID {vehicle.id})")
            if vehicle.destroy():
                 print("Vehicle destroyed successfully.")
            else:
                 print("Vehicle destroy() returned False.")
        else:
            print("Vehicle was None or not alive, no destruction needed.")
            
        # Optional: Disconnect client? Typically not needed unless managing multiple connections.
        # if client:
        #     pass # client doesn't have a disconnect method

        print("Simulation finished.")


if __name__ == '__main__':
    main()
