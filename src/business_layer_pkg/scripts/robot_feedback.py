#!/usr/bin/env python3
import rospy
import json
import subprocess
import re

from std_msgs.msg import String
from std_msgs.msg import Float32, Int16MultiArray, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu
from json_data import data

PI = 3.14159
wheel_radius = 0.24 # in meters

class RobotFeedback:
    def __init__(self,_ip):
        self.ip = _ip

        self.steering_health_check = False
        self.braking_health_check = False
        self.battery_capacity = 100
        self.ultrasonic_threshold = 100
        self.robot_velocity = 0
        self.robot_speed_feedback = 0
        self.steering_state = {
            "front right": "OK",
            "front left": "OK",
            "back right": "OK",
            "back left": "OK"
        }
        self.brake_state = {
            "front right": "OK",
            "front left": "OK",
            "back right": "OK",
            "back left": "OK"
        }
        self.ultrasonic = {
            "front_right": 0,
            "front_left": 0,
            "back_right": 0,
            "back_left": 0,
            "right": 0,
            "left": 0
        }

        self.motors_speed_sub           = rospy.Subscriber("feedback/motors_speed",   Int16MultiArray, self.motors_speed_cb)
        self.steering_angle_sub         = rospy.Subscriber("feedback/steering_angle",     Int16MultiArray, self.steering_angle_cb)
        self.brake_percentage_sub       = rospy.Subscriber("feedback/brake_percentage",   Int16MultiArray, self.brake_percentage_cb)
        self.ultrasonic_sub             = rospy.Subscriber("feedback/ultrasonic",     Int16MultiArray, self.ultrasonic_cb)
        self.rpy_sub                    = rospy.Subscriber("feedback/rpy",     Float32MultiArray, self.rpy_cb)

        self.battery_sub                = rospy.Subscriber("feedback/battery",    BatteryState, self.battery_cb)
        # self.imu_sub                    = rospy.Subscriber("feedback/imu",    Imu, self.imu_cb)
        self.robot_speed_sub            = rospy.Subscriber("feedback/robot_speed",    Float32, self.robot_speed_cb)
        self.door_state_sub             = rospy.Subscriber("feedback/door_state",     String, self.door_state_cb)
        self.steering_state_sub         = rospy.Subscriber("feedback/steering_state",    String, self.steering_state_cb)
        self.brake_state_sub            = rospy.Subscriber("feedback/brake_state",   String, self.brake_state_cb)
        # self.driving_mode_sub           = rospy.Subscriber("feedback/driving_mode",    String, self.driving_mode_cb)
        self.steering_health_check_sub  = rospy.Subscriber("feedback/steering_health_check",  Bool,  lambda msg : setattr(self, 'steering_health_check', msg.data))
        self.braking_health_check_sub   = rospy.Subscriber("feedback/braking_health_check",   Bool, lambda msg: setattr(self, 'braking_health_check', msg.data))



    def motors_speed_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['rpm'] = msg.data[0]
        data['motors_details']['fl']['rpm'] = msg.data[1]
        data['motors_details']['br']['rpm'] = msg.data[2]
        data['motors_details']['bl']['rpm'] = msg.data[3]

    def steering_angle_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['steering'] = msg.data[0]
        data['motors_details']['fl']['steering'] = msg.data[1]
        data['motors_details']['br']['steering'] = msg.data[2]
        data['motors_details']['bl']['steering'] = msg.data[3]

    def brake_percentage_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['brake'] = msg.data[0]
        data['motors_details']['fl']['brake'] = msg.data[1]
        data['motors_details']['br']['brake'] = msg.data[2]
        data['motors_details']['bl']['brake'] = msg.data[3]

    def ultrasonic_cb(self, msg: Int16MultiArray):
        
        #convert the ultrasonic data to binary
        self.ultrasonic['front_right']  = 1 if msg.data[0] < self.ultrasonic_threshold and msg.data[0] > 0 else 0
        self.ultrasonic['front_left']   = 1 if msg.data[1] < self.ultrasonic_threshold and msg.data[1] > 0 else 0
        self.ultrasonic['back_right']   = 1 if msg.data[2] < self.ultrasonic_threshold and msg.data[2] > 0 else 0
        self.ultrasonic['back_left']    = 1 if msg.data[3] < self.ultrasonic_threshold and msg.data[3] > 0 else 0
        self.ultrasonic['right']        = 1 if msg.data[4] < self.ultrasonic_threshold and msg.data[4] > 0 else 0
        self.ultrasonic['left']         = 1 if msg.data[5] < self.ultrasonic_threshold and msg.data[5] > 0 else 0
        
        data['surroundings']['front_center']    = self.ultrasonic['front_right'] or self.ultrasonic['front_left']
        data['surroundings']['front_right']     = self.ultrasonic['front_right'] or self.ultrasonic['right']
        data['surroundings']['front_left']      = self.ultrasonic['front_left']  or self.ultrasonic['left']
        
        data['surroundings']['back_center']     = self.ultrasonic['back_right'] or self.ultrasonic['back_left']
        data['surroundings']['back_right']      = self.ultrasonic['back_right'] or self.ultrasonic['right']
        data['surroundings']['back_left']       = self.ultrasonic['back_left']  or self.ultrasonic['left']
    
    def rpy_cb(self, msg: Int16MultiArray):
        data['orientation']['roll']     = msg.data[0]
        data['orientation']['pitch']    = msg.data[1]
        data['orientation']['yaw']      = msg.data[2]

    def battery_cb(self, msg: BatteryState):
        self.battery_capacity = msg.percentage
        data['battery']['percentage'] = msg.percentage
        data['temperature'] = msg.temperature

    def robot_speed_cb(self, msg:Float32):
        self.robot_speed_feedback = msg.data
        speed_kmh = (self.robot_speed_feedback * 2 * PI * wheel_radius * 60) / 1000
        data['speed'] = int (speed_kmh)

    def door_state_cb(self, msg:String):
        data['door_state'] = msg.data
        # print("door state: ", data['door_state'])

    def steering_state_cb(self, msg:String):
        split_data = msg.data.split(':')
        # print("steering_state: ", split_data)
        self.steering_state['front right']  = split_data[0]
        self.steering_state['front left']   = split_data[1]
        self.steering_state['back right']   = split_data[2]
        self.steering_state['back left']    = split_data[3]
        # print("steering_state: ", self.steering_state)
            
    def brake_state_cb(self, msg:String):
        split_data = msg.data.split(':')
        self.brake_state['front right']  = split_data[0]
        self.brake_state['front left']   = split_data[1]
        self.brake_state['back right']   = split_data[2]
        self.brake_state['back left']    = split_data[3]

    def driving_mode_cb(self, msg:String):
        data['drivingMode'] = msg.data

    def updateDriveMode(self, robot_velocity):
        self.robot_velocity = robot_velocity
        
        if self.robot_velocity > 0 or self.robot_speed_feedback > 0:
            data['drivingMode'] = "D"
        
        elif self.robot_velocity < 0 or self.robot_speed_feedback < 0:
            data['drivingMode'] = "R"
        
        elif self.robot_velocity == 0 or self.robot_speed_feedback == 0:
            data['drivingMode'] = "P"
    
    def getEmergencyCause(self):
        emergency_cause = []

        if self.battery_capacity < 20:
            emergency_cause.append("low battery")
        # Check for errors in steering state
        for motor_type, state in self.steering_state.items():
            if state != "ok":
                emergency_cause.append(f"steering motor:{motor_type} {state}")

        # Check for errors in brake state
        for motor_type, state in self.brake_state.items():
            if state != "ok":
                emergency_cause.append(f"brake motor:{motor_type} {state}")

        # If no errors found, return an empty string
        if not emergency_cause:
            return ""

        # Return all collected error causes as a single string
        return "; ".join(emergency_cause)

    def getBatteryCapacity(self):
        return self.battery_capacity

    def getHealthCheck(self):
        # print("steering_health_check: ", self.steering_health_check)
        # print("braking_health_check: ", self.braking_health_check)
        return self.steering_health_check, self.braking_health_check
    
    def ping_host(self,ip, count=1):
        try:
            # Run the ping command
            output = subprocess.check_output(['ping', '-c', str(count), ip], universal_newlines=True)
            
            # Parse the output
            # Extract the average round-trip time (RTT)
            rtt_match = re.search(r'rtt min/avg/max/mdev = .*?/([0-9.]+)/', output)
            avg_rtt = float(rtt_match.group(1)) if rtt_match else None
            
            # Extract packet loss percentage
            loss_match = re.search(r'(\d+)% packet loss', output)
            packet_loss = int(loss_match.group(1)) if loss_match else None
            
            return avg_rtt, packet_loss

        except subprocess.CalledProcessError as e:
            print("Failed to ping host:", e)
            return None, None
    
    def calculate_connection_quality(self,avg_rtt, packet_loss):
        # Initial connection quality percentage
        quality = 100

        # Decrease quality based on packet loss
        if packet_loss:
            quality -= packet_loss * 1.5  # Packet loss has a significant impact

        # Decrease quality based on RTT
        if avg_rtt:
            if avg_rtt < 100:
                quality -= avg_rtt * 0.1  # Minor impact for low RTT
            elif avg_rtt < 300:
                quality -= avg_rtt * 0.2  # Moderate impact for medium RTT
            else:
                quality -= avg_rtt * 0.5  # High impact for high RTT

        # Ensure quality is within 0-100 range
        quality = max(0, min(100, quality))

        return quality

    def getConnectionQuality(self):
        avg_rtt, packet_loss = self.ping_host(self.ip)
        if avg_rtt is not None and packet_loss is not None:
            # print(f"Average RTT: {avg_rtt} ms")
            # print(f"Packet Loss: {packet_loss}%")
            connection = self.calculate_connection_quality(avg_rtt, packet_loss)
            return connection
        return 0

    def getRobotFeedback(self, elapsed_time):
        data['duration']['elapsed'] = elapsed_time
        data['timestamp'] = rospy.get_time()
        data['connection_quality'] = self.getConnectionQuality()
        
        json_string = json.dumps(data)

        return json_string


if __name__ == "__main__":
    try:
        robot = RobotFeedback()
        test_data = "ok:ok:MotorError:MotorError"
        fr_state, fl_state, br_state, bl_state = test_data.split(':')

        print("fr_state: ", fr_state)
        print("fl_state: ", fl_state)
        print("br_state: ", br_state)
        print("bl_state: ", bl_state)
        while not rospy.is_shutdown():
            # robot.priority_control()
            rospy.sleep(0.03)
        

    except rospy.ROSInterruptException:
        pass
