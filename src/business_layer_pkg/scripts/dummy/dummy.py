#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from random import randint

data = {
    "lightning": {
        "siren": 1,  # 0
        "headlights": 0,  # 0
        "left": 0,  # 0
        "right": 1,  # 0
    },
    "surroundings": {
        "front_center": 0,  # 1
        "back_center": 0,  # 1
        "front_right": 1,  # 1
        "back_right": 1,  # 1
        "front_left": 0,  # 1
        "back_left": 0,  # 1
    },
    "gps": {
        "long": 55.21321,
        "lat": 21.213234,
        "alt": 13.345,
    },
    "connection_quality": 80,  # percentage
    "distance": {
        # KM
        "travelled": 12,
        "remaining": 12,
        "planned": 12,
        "on_charge": 12,
        "percentage": 12,
    },
    "duration": {
        # Time format 24
        "elapsed": "01:21:55",
        "remaining": "00:42:12",
        "estimated": "02:04:07",
    },
    "laps": {"elapsed": 3, "planned": 5},
    "battery": {"needed": 23, "percentage": 55},
    "temperature": 123,
    "odometer": 15487,  # KM
    "speed": 3,  # KM/H,
    "drivingMode": "P",  # D,R
    "orientation": {"roll": 12.00, "pitch": 12.00, "yaw": 12.88},
    "motors_details": {
        "fr": {
            "rpm": 23,  # int
            "steering": 18,  # int
            "brake": 12,  # int
            "torque": 13,  # float
            "power": 23.434,
        },
        "fl": {
            "rpm": 23,  # int
            "steering": -18,  # int
            "brake": 12,  # int
            "torque": 13,  # float
            "power": 23.434,
        },
        "br": {
            "rpm": 23,  # int
            "steering": 18,  # int
            "brake": 12,  # int
            "torque": 13,  # float
            "power": 23.434,
        },
        "bl": {
            "rpm": 23,  # int
            "steering": -18,  # int
            "brake": 12,  # int
            "torque": 13,  # float
            "power": 23.434,
        },
    },
    "door_state": "closing",
    "timestamp": 0,
}

steeringError = [
    "OK",
    "MOTOR_OVER_TEMPERATURE",
    "MOTOR_OVER_CURRENT",
    "POSITION_SENSOR_DAMAGED",
    "POSITION_SENSOR_RESPONSE",
    "MOTOR_ERROR",
    "MOTOR_CONNECTION_LOSS",
]

brakingError = [
    "OK",
    "MOTOR_OVER_TEMPERATURE",
    "MOTOR_OVER_CURRENT",
    "MOTOR_ERROR",
    "MOTOR_CONNECTION_LOSS",
]

emergency_causes = {
    "steering": {
        "front right": "OK",
        "front left": "OK",
        "back right": "OK",
        "back left": "OK",
    },
    "brake": {
        "front right": "OK",
        "front left": "OK",
        "back right": "OK",
        "back left": "OK",
    },
}


def gear_cb(msg):
    print("Gear: ", msg.data)


def wasdb_cb(msg):
    print("WASDB: ", msg.data)


def door_control_cb(msg):
    print("door_control: ", msg.data)


def req_streames_cb(req):
    streams_ids = {
        "streams": [
            {
                "name": "Front Camera",
                "type": "front",
                "connectionId": "dae8360a-829c-4e67-b32a-70d4e5b90353",
            },
            {
                "name": "Back Camera",
                "type": "back",
                "connectionId": "c2f1cde9-aabe-4f75-b764-bd54473d8c53",
            },
        ]
    }

    return TriggerResponse(success=True, message=json.dumps(streams_ids))


def publisher_node():
    # Initialize the ROS node
    rospy.init_node("dummy_py", anonymous=True)

    # Create a publisher object that will publish on the 'json_data' topic
    pub = rospy.Publisher("robot_operational_details", String, queue_size=1)
    emergency_pub = rospy.Publisher("emergency_cause", String, queue_size=1)

    gear_sub = rospy.Subscriber("gear", String, gear_cb)
    wasdb_sub = rospy.Subscriber("wasdb", String, wasdb_cb)
    door_control_sub = rospy.Subscriber("door_control", String, door_control_cb)
    streams_srv = rospy.Service("streams_ids", Trigger, req_streames_cb)

    # Set the rate of publishing
    rate = rospy.Rate(1)  # 1 Hz

    # Define the JSON object to be sent
    i=0
    j=0
    while not rospy.is_shutdown():
        # Update the timestamp for each message
        data["timestamp"] = rospy.get_time()
        
        emergency_causes["steering"]["front right"] = steeringError[i]
        emergency_causes["brake"]["front right"] = brakingError[j]
        i+=1
        j+=1
        if i == 7:
            i = 0
        if j == 5:
            j = 0
        # Convert the JSON object to a string
        json_string = json.dumps(data)
        json_emergency = json.dumps(emergency_causes)
        

        # Publish the JSON string
        pub.publish(json_string)
        emergency_pub.publish(json_emergency)


        # Sleep to maintain the publishing rate
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
