#!/usr/bin/env python3

import os
import subprocess

def check_ros_version():
    # Check for ROS1
    ros_dist = os.getenv('ROS_DISTRO')
    if ros_dist:
        return f"ROS is installed with distribution: {ros_dist}"
    
    # Check for ROS2
    try:
        result = subprocess.run(['ros2', '--version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        if result.returncode == 0:
            return "ROS2 is installed"
    except FileNotFoundError:
        pass
    
    return "Neither ROS1 nor ROS2 is installed"

if __name__ == "__main__":
    version_info = check_ros_version()
    print(version_info)
