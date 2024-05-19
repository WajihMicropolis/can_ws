#!/usr/bin/env python3

data = {
    "lightning": {
        "siren" :  1, # 0
        "headlights" : 0, # 0
        "left" :  0, # 0
        "right" : 1 # 0
    },
    "surroundings": {
        "front_center" :0, # 1
        "back_center" :  0, # 1
        "front_right" : 1, # 1
        "back_right" :  1, # 1
        "front_left" : 0, # 1
        "back_left" :  0 # 1
    },
    "gps": {
        "long": 55.21321,
        "lat": 21.213234,
        "alt": 13.345,
    },
    "connection_quality": 80, # percentage
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
        "elapsed": "01:21:55",
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
    "door_state":"closed", #?done
    "steering_status": "ok",#?done
    "brake_status": "ok",#?done
    "timestamp": 0
}