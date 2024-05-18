#!/usr/bin/env python3

from pynput import keyboard
from copy import deepcopy
import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Int8

drive_data = {"wasdb": {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}}  # 0  # 0  # 0  # 0
prev_drive_data = deepcopy(drive_data)

gear_data = {
    "gear": 2,
}
prev_gear = deepcopy(gear_data)


def on_press(key):
    global drive_data, prev_drive_data
    try:
        # print("alphanumeric key {0} pressed".format(key.char))
        if key.char == "w":
            drive_data["wasdb"]["w"] = 1
            drive_data["wasdb"]["s"] = 0
        elif key.char == "s":
            drive_data["wasdb"]["s"] = 1
            drive_data["wasdb"]["w"] = 0
        elif key.char == "a":
            drive_data["wasdb"]["a"] = 1
            drive_data["wasdb"]["d"] = 0
        elif key.char == "d":
            drive_data["wasdb"]["d"] = 1
            drive_data["wasdb"]["a"] = 0
            
    except AttributeError:
        # print("special key {0} pressed".format(key))
        if key == keyboard.Key.space:
            drive_data["wasdb"]["b"] = 1
            drive_data["wasdb"]["s"] = 0
            drive_data["wasdb"]["w"] = 0
            drive_data["wasdb"]["d"] = 0
            drive_data["wasdb"]["a"] = 0
        
    
    if drive_data != prev_drive_data:
        prev_drive_data = deepcopy(drive_data)
        wasdb_pub.publish(convert_to_json(drive_data))


def on_release(key):
    global drive_data, prev_drive_data
    global gear_data, prev_gear
    
    try:
        # print("{0} released".format(key))
        if key.char == "w":
            drive_data["wasdb"]["w"] = 0
        elif key.char == "s":
            drive_data["wasdb"]["s"] = 0
        elif key.char == "a":
            drive_data["wasdb"]["a"] = 0
        elif key.char == "d":
            drive_data["wasdb"]["d"] = 0
        elif key.char == "b":
            drive_data["wasdb"]["b"] = 0
    
    except AttributeError:
        if key == keyboard.Key.space:
            drive_data["wasdb"]["b"] = 0
            drive_data["wasdb"]["s"] = 0
            drive_data["wasdb"]["w"] = 0
            drive_data["wasdb"]["d"] = 0
            drive_data["wasdb"]["a"] = 0
            
        if key == keyboard.Key.up:
            gear_data["gear"] += 2
            gear_data["gear"] = min(10, gear_data["gear"])
            
        if key == keyboard.Key.down:
            gear_data["gear"] -= 2
            gear_data["gear"] = max(2, gear_data["gear"])
    
    if gear_data != prev_gear:
        prev_gear = deepcopy(gear_data)
        gear_pub.publish(convert_to_json(gear_data))
            
    if drive_data != prev_drive_data:
        prev_drive_data = deepcopy(drive_data)
        wasdb_pub.publish(convert_to_json(drive_data))
    
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def convert_to_json(data):
    return json.dumps(data)


if __name__ == "__main__":
    try:
        # Initialize the ROS node
        rospy.init_node("dummy_py", anonymous=True)

        gear_pub = rospy.Publisher("gear", String, queue_size=1)
        wasdb_pub = rospy.Publisher("wasdb", String, queue_size=1)
        # Collect events until released
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
    except rospy.ROSInterruptException:
        pass
