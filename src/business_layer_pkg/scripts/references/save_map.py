#!/usr/bin/env python3
import subprocess

def save_map(map_file_path):
    try:
        # Define the command to source the ROS environment and run map_saver
        
        command = f"bash -c ' rosrun map_server map_saver -f {map_file_path} map:=/projected_map'"
        
        # Run the command in a shell to allow environment sourcing
        result = subprocess.run(command, capture_output=True, text=True, shell=True,timeout=3)
        
        # Check the output for success message
        if 'Done' in result.stdout:
            print("Map saved successfully.")
        else:
            print("Map saving failed. Check the output for details.")
            print("stdout:", result.stdout)
            print("stderr:", result.stderr)
    
    except subprocess.CalledProcessError as e:
        print(f"Failed to save the map: {e}")
        print(e.output)

if __name__ == "__main__":
    map_file_path = "/home/microspot/catkin_ws/maps/map"  # Replace with your desired map file path
    save_map(map_file_path)
