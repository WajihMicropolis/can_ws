#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from random import randint
import requests

data = {
    "lightning": {
        "siren" :  0, # 0
        "headlights" : 0, # 0
        "left" :  0, # 0 #?done
        "right" : 0 # 0 #?done
    },
    "surroundings": { #?done
        "front_center" :0, # 1
        "back_center" :  0, # 1
        "front_right" : 0, # 1
        "back_right" :  0, # 1
        "front_left" : 0, # 1
        "back_left" :  0 # 1
    },
    "gps": {
        "long": 55.21321,
        "lat": 21.213234,
        "alt": 13.345,
    },
    "connection_quality": 100, # percentage #?done
    "distance": {
        # KM
        "travelled": 12,
        "remaining": 12,
        "planned": 12,
        "on_charge": 12,
        "percentage":12
    },
    "duration": {
        # Time format 24
        "elapsed": "00:00:00", #?done
        "remaining": "00:42:12",
        "estimated": "02:04:07"
    },
    "laps": {
        "elapsed": 3,
        "planned": 5
    },
    "battery": { 
        "needed": 23, 
        "percentage": 100 #?done
    },
    "temperature": 123, 
    "odometer": 15487, #KM
    "speed": 0, #KM/H, #?done
    "drivingMode" : "P", # D,R #?done
    "orientation" : { #?done
        "roll" : 0.00,
        "pitch": 0.00,
        "yaw": 0.0
    },
    "motors_details": { #?done
        "fr": {
        "rpm": 0, # int
        "steering": 0, #int
        "brake": 10, # int
        "torque": 13, # float
        "power": 23.434
        },
        "fl": {
        "rpm": 0, # int
        "steering": 0, #int
        "brake": 10, # int
        "torque": 13, # float
        "power": 23.434
        },
        "br": {
        "rpm": 0, # int
        "steering": 0, #int
        "brake": 10, # int
        "torque": 13, # float
        "power": 23.434
        },
        "bl": {
        "rpm": 0, # int
        "steering": 0, #int
        "brake": 10, # int
        "torque": 13, # float
        "power": 23.434
        }
    },
    
    "door_state":{
        "top":"closed",
        "front_right": "closed",
        "front_left": "closed",
        "back_right": "closed",
        "back_left": "closed",
    },
    
    "timestamp": 0 #?done
    
}

pod_doors_control = {
    "front_right": 0,
    "front_left": 0,
    "back_right": 0,
    "back_left": 0
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
    "low_battery": True # False
}

door_control = {
    "target_door":"top",
    "target_state": "open"
}

def get_public_ip():
    try:
        response = requests.get('https://api.ipify.org?format=json')
        response.raise_for_status()  # Raise an exception for HTTP errors
        ip_info = response.json()
        return ip_info['ip']
    except requests.RequestException as e:
        print(f"Error occurred: {e}")
        return None
    
def gear_cb(msg:String):        emergency_causes["low_battery"] = randint(0, 1)


def wasdb_cb(msg:String):
    print("WASDB: ", msg.data)


def door_control_cb(msg:String):
    door_control = json.loads(msg.data)
    print("target_door: ", door_control["target_door"])
    print("target_state: ", door_control["target_state"])
    # print("door_control: ", msg.data)
    pass

def req_streames_cb(req):
    public_ip = get_public_ip()
    print("public ip: ", public_ip)
    streams_ids = {
        "streams": [
            { 
                "name": "Front Camera",
                "type": "front", 
                "connectionId": {
                    "address": public_ip,
                    "port": 8555,
                    "stream_name": "front_camera"
                }
            },
            {
                "name": "Back Camera", 
                "type": "back", 
                "connectionId": {
                    "address": public_ip,
                    "port": 8556,
                    "stream_name": "back_camera"
                }
            }
        ]
    }

    return TriggerResponse(success=True, message=json.dumps(streams_ids))

def doors_select_cb(msg:String):
    doors = json.loads(msg.data)
    print("doors: ", doors)
    # print("fron_right:", doors["front_right"])
    
def publisher_node():
    # Initialize the ROS node
    rospy.init_node("dummy_py", anonymous=True)

    # Create a publisher object that will publish on the 'json_data' topic
    pub = rospy.Publisher("robot_operational_details", String, queue_size=1, latch=True)
    pod_pub = rospy.Publisher("pod_details", String, queue_size=1, latch=True)
    emergency_pub = rospy.Publisher("emergency_cause", String, queue_size=1, latch=True)

    gear_sub = rospy.Subscriber("gear", String, gear_cb)
    wasdb_sub = rospy.Subscriber("wasdb", String, wasdb_cb)
    door_control_sub = rospy.Subscriber("door_control", String, door_control_cb)
    streams_srv = rospy.Service("streams_ids", Trigger, req_streames_cb)
    doors_select_sub = rospy.Subscriber("pod_doors_control", String, doors_select_cb)

    # Set the rate of publishing
    rate = rospy.Rate(1)  # 1 Hz

    # Define the JSON object to be sent
    i=0
    j=0
    while not rospy.is_shutdown():
        print("Publishing...")
        # Update the timestamp for each message
        data["timestamp"] = rospy.get_time()
        
        emergency_causes["steering"]["front right"] = steeringError[i]
        emergency_causes["brake"]["front right"] = brakingError[j]
        emergency_causes["low_battery"] = randint(0, 1)
        i+=1
        j+=1
        if i == 7:
            i = 0
        if j == 5:
            j = 0
        # Convert the JSON object to a string
        json_string = json.dumps(data)
        # json_pod = json.dumps(pod_details)
        json_emergency = json.dumps(emergency_causes)
        

        # Publish the JSON string
        pub.publish(json_string)
        # pod_pub.publish(json_pod)
        emergency_pub.publish(json_emergency)


        # Sleep to maintain the publishing rate
        rate.sleep()


if __name__ == "__main__":
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
