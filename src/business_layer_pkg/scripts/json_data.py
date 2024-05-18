#!/usr/bin/env python3
import json

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
        "percentage": 55
    },
    "temperature": 123,
    "odometer": 15487, #KM
    "speed": 3, #KM/H,
    "drivingMode" : "P", # D,R
    "orientation" : {
        "roll" : 12.00,
        "pitch": 12.00,
        "yaw": 12.88
    },
    "motors_details": {
        "fr": {
        "rpm": 23, # int
        "steering": 18, #int
        "brake": 12, # int
        "torque": 13, # float
        "power": 23.434
        },
        "fl": {
        "rpm": 23, # int
        "steering": -18, #int
        "brake": 12, # int
        "torque": 13, # float
        "power": 23.434
        },
        "br": {
        "rpm": 23, # int
        "steering": 18, #int
        "brake": 12, # int
        "torque": 13, # float
        "power": 23.434
        },
        "bl": {
        "rpm": 23, # int
        "steering": -18, #int
        "brake": 12, # int
        "torque": 13, # float
        "power": 23.434
        }
    },
    "door_state":"closed",
    "steering_status": "ok",
    "brake_status": "ok",
    "timestamp": 0
}