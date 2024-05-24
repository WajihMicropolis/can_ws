#!/usr/bin/env python3

from copy import deepcopy
from random import randint
import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Int8, Int16MultiArray

drive_data = {"wasdb": {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}}  # 0  # 0  # 0  # 0
prev_drive_data = deepcopy(drive_data)

gear_data = {
    "gear": 1,
}
prev_gear = deepcopy(gear_data)

door_data = {
    "state": "closed",
}
prev_door = deepcopy(door_data)

def dir_cb(msg: String):
    global drive_data, prev_drive_data
    # prev_drive_data = deepcopy(drive_data)
    print("dir: ", msg.data)
    if msg.data == "w":
        drive_data["wasdb"]["w"] = not drive_data["wasdb"]["w"]
        drive_data["wasdb"]["s"] = 0
    elif msg.data == "s":
        drive_data["wasdb"]["s"] = not drive_data["wasdb"]["s"]
        drive_data["wasdb"]["w"] = 0

    elif msg.data == "a":
        drive_data["wasdb"]["a"] = not drive_data["wasdb"]["a"]
        drive_data["wasdb"]["d"] = 0
    elif msg.data == "d":
        drive_data["wasdb"]["d"] = not drive_data["wasdb"]["d"]
        drive_data["wasdb"]["a"] = 0

    elif msg.data == "b":
        drive_data["wasdb"]["b"] = not drive_data["wasdb"]["b"]

    print("drive_data: ", drive_data)


def gear_change_cb(msg: Int8):

    print("gear_change: ", msg.data)
    gear_data["gear"] = msg.data


def publisher_node():
    # Initialize the ROS node
    rospy.init_node("dummy_py", anonymous=True)

    direction_sub = rospy.Subscriber("dir", String, dir_cb)
    gear_change_sub = rospy.Subscriber("gear_change", Int8, gear_change_cb)
    # Create a publisher object that will publish on the 'json_data' topic
    gear_pub = rospy.Publisher("gear", String, queue_size=1)
    wasdb_pub = rospy.Publisher("wasdb", String, queue_size=1)
    door_control_pub = rospy.Publisher("door_control", String, queue_size=1)
    us = rospy.Publisher("feedback/ultrasonic", Int16MultiArray, queue_size=1)
    
    # Set the rate of publishing
    rate = rospy.Rate(1)  # 1 Hz

    # Define the JSON object to be sent

    global drive_data, prev_drive_data
    global gear_data, prev_gear
    global door_data, prev_door
    us_data = Int16MultiArray()
    while not rospy.is_shutdown():
        # Convert the JSON object to a string
        wasdb_json_string = json.dumps(drive_data)
        gear_json_string = json.dumps(gear_data)

        # Publish the JSON string
        if prev_drive_data != drive_data:
            wasdb_pub.publish(wasdb_json_string)
            prev_drive_data = deepcopy(drive_data)

        print("gear_data: ", gear_data)
        print("prev_gear: ", prev_gear)
        if prev_gear != gear_data:
            gear_pub.publish(gear_json_string)
            prev_gear = deepcopy(gear_data)
        
        door_data = {
            "state": "open" if randint(0, 1) else "closed"
        }
        us_data.data = [0,10,20,30,40,50]
        us.publish(us_data)
        
        if prev_door != door_data:
            door_control_pub.publish(json.dumps(door_data))
            prev_door = deepcopy(door_data)
        # Sleep to maintain the publishing rate
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
