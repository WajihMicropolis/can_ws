#!/usr/bin/env python3
from copy import deepcopy
import rospy
import json
import subprocess
import re
import datetime

from std_msgs.msg import String
from std_msgs.msg import Float32, Int16MultiArray, Bool, Float32MultiArray, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu
from json_data import data

PI = 3.14159
wheel_radius = 0.24 # in meters

class RobotFeedback:
    def __init__(self,_ip):
        self.ip = _ip

        self.steering_health_check = True
        self.braking_health_check = True
        self.battery_percentage = 100
        self.ultrasonic_threshold = 100
        self.robot_velocity = 0
        self.robot_speed_feedback = 0
        self.connection_quality = 100

        self.ultrasonic = {
            "front_right": 0,
            "front_left": 0,
            "back_right": 0,
            "back_left": 0,
            "right": 0,
            "left": 0
        }

        self.emergency_causes = {
            "steering": {
                "front right":  "OK",
                "front left":   "OK",
                "back right":   "OK",
                "back left":    "OK",
            },
            "brake": {
                "front right":  "OK",
                "front left":   "OK",
                "back right":   "OK",
                "back left":    "OK",
            },
            "low_battery": True # False
        }
        # self.pod_details = {
        #     "doors_state":{
        #         "front_right": "closed",
        #         "front_left": "closed",
        #         "back_right": "closed",
        #         "back_left": "closed",
        #     }
        # }
        self.prev_emergency_causes = deepcopy(self.emergency_causes)
        self.steering_state_count = 0
        self.brake_state_count = 0
        self.battery_state_count = 0
        self.emergency_cause_msg = String()
        self.emergency_cause_msg.data = json.dumps(self.emergency_causes)
        self.light_timer = rospy.Time.now()
        self.init_time = rospy.Time.now().to_sec()
        self.elapsed_time = "00:00:00"
        
        self.door_state = "closed"
        self.lifter_state = "closed"
        self.drone_base_state = "closed"
        
        self.motors_speed_sub           = rospy.Subscriber("feedback/motors_speed",   Int16MultiArray, self.motors_speed_cb)
        self.steering_angle_sub         = rospy.Subscriber("feedback/steering_angle",     Int16MultiArray, self.steering_angle_cb)
        self.brake_percentage_sub       = rospy.Subscriber("feedback/brake_percentage",   Int16MultiArray, self.brake_percentage_cb)
        self.ultrasonic_sub             = rospy.Subscriber("feedback/ultrasonic",     Int16MultiArray, self.ultrasonic_cb)
        self.rpy_sub                    = rospy.Subscriber("feedback/rpy",     Float32MultiArray, self.rpy_cb)

        self.battery_sub                = rospy.Subscriber("feedback/battery",    BatteryState, self.battery_cb)
        # self.imu_sub                    = rospy.Subscriber("feedback/imu",    Imu, self.imu_cb)
        self.robot_speed_sub            = rospy.Subscriber("feedback/robot_speed",    Float32, self.robot_speed_cb)
        self.door_state_sub             = rospy.Subscriber("feedback/door_state",     String, lambda msg : setattr(self, 'door_state', msg.data))
        self.lifter_state_sub           = rospy.Subscriber("feedback/lifter_state",     String, lambda msg : setattr(self, 'lifter_state', msg.data))
        self.drone_base_state_sub       = rospy.Subscriber("feedback/drone_base_state",     String, lambda msg : setattr(self, 'drone_base_state', msg.data))
        self.pod_doors_state_sub        = rospy.Subscriber("feedback/pod_doors_state",     String, self.pod_doors_state_cb)
        
        self.steering_state_sub         = rospy.Subscriber("feedback/steering_state",    String, self.steering_state_cb)
        self.brake_state_sub            = rospy.Subscriber("feedback/brake_state",   String, self.brake_state_cb)
        self.steering_health_check_sub  = rospy.Subscriber("feedback/steering_health_check",  Bool,  lambda msg : setattr(self, 'steering_health_check', msg.data))
        self.braking_health_check_sub   = rospy.Subscriber("feedback/braking_health_check",   Bool, lambda msg: setattr(self, 'braking_health_check', msg.data))


# ! CALLBACKS
    def motors_speed_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['rpm'] = abs(msg.data[0])
        data['motors_details']['fl']['rpm'] = abs(msg.data[1])
        data['motors_details']['br']['rpm'] = abs(msg.data[2])
        data['motors_details']['bl']['rpm'] = abs(msg.data[3])

    def steering_angle_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['steering'] = msg.data[0]
        data['motors_details']['fl']['steering'] = msg.data[1]
        data['motors_details']['br']['steering'] = msg.data[2]
        data['motors_details']['bl']['steering'] = msg.data[3]

    def brake_percentage_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['brake'] = msg.data[0] * 10
        data['motors_details']['fl']['brake'] = msg.data[1] * 10
        data['motors_details']['br']['brake'] = msg.data[2] * 10
        data['motors_details']['bl']['brake'] = msg.data[3] * 10

    def ultrasonic_cb(self, msg: Int16MultiArray):
        
        #convert the ultrasonic data to binary
        self.ultrasonic['front_right']  = 1 if msg.data[0] < self.ultrasonic_threshold and msg.data[0] > 40 else 0
        self.ultrasonic['front_left']   = 1 if msg.data[1] < self.ultrasonic_threshold and msg.data[1] > 40 else 0
        self.ultrasonic['back_right']   = 1 if msg.data[2] < self.ultrasonic_threshold and msg.data[2] > 40 else 0
        self.ultrasonic['back_left']    = 1 if msg.data[3] < self.ultrasonic_threshold and msg.data[3] > 40 else 0
        self.ultrasonic['right']        = 1 if msg.data[4] < self.ultrasonic_threshold and msg.data[4] > 40 else 0
        self.ultrasonic['left']         = 1 if msg.data[5] < self.ultrasonic_threshold and msg.data[5] > 40 else 0
        
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

    def pod_doors_state_cb(self, msg: String):
        if not msg.data:
            return
        split_data = msg.data.split(':')
        data['door_state']['front_right'] = split_data[0]
        data['door_state']['front_left'] = split_data[1]
        data['door_state']['back_right'] = split_data[2]
        data['door_state']['back_left'] = split_data[3]

    # def door_state_cb(self, msg:String):
    #     self.door_state = msg.data
    
    # def lifter_state_cb(self, msg:String):
    #     self.lifter_state = msg.data
        
    # def drone_base_cb(self, msg:String):
    #     self.drone_base_state = msg.data
    
    def robot_speed_cb(self, msg:Float32):
        self.robot_speed_feedback = msg.data
        # convert the speed from rpm to km/h
        speed_kmh = abs(self.robot_speed_feedback * 2 * PI * wheel_radius * 60) / 1000
        data['speed'] = int (speed_kmh)
        
    def battery_cb(self, msg: BatteryState):
        if not msg.voltage:
            return
        self.battery_percentage = msg.percentage
        data['battery']['percentage'] = msg.percentage
        data['temperature'] = msg.temperature
        self.checkBatteryState()

    def steering_state_cb(self, msg:String):
        if not msg.data:
            return
        split_data = msg.data.split(':')
        # print("steering_state: ", split_data)
        self.emergency_causes["steering"]['front right']  = split_data[0]
        self.emergency_causes["steering"]['front left']   = split_data[1]
        self.emergency_causes["steering"]['back right']   = split_data[2]
        self.emergency_causes["steering"]['back left']    = split_data[3]
        self.checkErrorState()
        
        # print("steering_state: ", self.steering_state)
            
    def brake_state_cb(self, msg:String):
        if not msg.data:
            return
        
        split_data = msg.data.split(':')
        self.emergency_causes["brake"]['front right']  = split_data[0]
        self.emergency_causes["brake"]['front left']   = split_data[1]
        self.emergency_causes["brake"]['back right']   = split_data[2]
        self.emergency_causes["brake"]['back left']    = split_data[3]
        self.checkErrorState()

# ! Get and Update Data
    def checkBatteryState(self):
        if self.battery_percentage < 30:
            self.battery_state_count += 1
            if self.battery_state_count > 20 and self.emergency_causes["low_battery"] == False:
                self.emergency_causes["low_battery"] = True
                self.emergency_cause_msg.data = json.dumps(self.emergency_causes)
                self.battery_state_count = 0
                
                
    def updateDoorState(self):
        
        if self.door_state == "opening" and self.lifter_state == "closed" and self.drone_base_state == "closed":
            data['door_state']["top"] = "opening"
            
            
        elif self.door_state == "opened" and self.lifter_state == "opened" and self.drone_base_state == "opened":
            data['door_state']["top"] = "opened"
        
        elif self.door_state == "opened" and self.lifter_state == "opened" and self.drone_base_state == "closing":
            data['door_state']["top"] = "closing"
            
        elif self.door_state == "closed" and self.lifter_state == "closed" and self.drone_base_state == "closed":
            data['door_state']["top"] = "closed"

    def checkErrorState(self):
        
        if self.emergency_causes["steering"] != self.prev_emergency_causes["steering"]:
            self.steering_state_count += 1
            
            if self.steering_state_count > 20:
                self.emergency_cause_msg.data = json.dumps(self.emergency_causes)
                self.steering_state_count = 0
                self.prev_emergency_causes["steering"] = deepcopy(self.emergency_causes["steering"])
                
        if self.emergency_causes["brake"] != self.prev_emergency_causes["brake"]:
            self.brake_state_count += 1
            
            if self.brake_state_count > 20:
                self.emergency_cause_msg.data = json.dumps(self.emergency_causes)
                self.brake_state_count = 0
                self.prev_emergency_causes["brake"] = deepcopy(self.emergency_causes["brake"])
        

    def updateDriveMode(self, robot_velocity):
        self.robot_velocity = robot_velocity

        if self.robot_velocity > 0 or self.robot_speed_feedback > 0:
            data['drivingMode'] = "D"
        
        elif self.robot_velocity < 0 or self.robot_speed_feedback < 0:
            data['drivingMode'] = "R"
        
        elif self.robot_velocity == 0 or self.robot_speed_feedback == 0:
            data['drivingMode'] = "P"
    
    def updateDirectionLight(self, direction):
        
        if direction < 1 and direction > -1:
            data['lightning']['left'] = 0
            data['lightning']['right'] = 0
            return
        if rospy.Time.now() - self.light_timer < rospy.Duration(0.3):
            return
        self.light_timer = rospy.Time.now()
        if direction > 1:
            data['lightning']['left'] = not data['lightning']['left']
            data['lightning']['right'] = 0
        
        elif direction < -1:
            data['lightning']['left'] = 0
            data['lightning']['right'] = not data['lightning']['right']
        
        

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

    def getEmergencyCause(self):
        return self.emergency_cause_msg.data

    def getBatteryCapacity(self):
        return self.battery_percentage

    def getHealthCheck(self):
        return self.steering_health_check, self.braking_health_check
    
    def update_elapsed_time(self, robot_state):
        if robot_state == "KEY_OFF":
            return
        t = rospy.Time.now().to_sec()
        start_time = t - self.init_time
        start_time = round(start_time, 2)
        elapsed_time = datetime.timedelta(seconds=int(start_time))
        time_str = str(elapsed_time)
        # print("time: ", time_str)
        return time_str
    
    def getRobotDetails(self, connection_quality,robot_state):
        connection_quality = self.getConnectionQuality()
        
        data['duration']['elapsed'] = self.update_elapsed_time(robot_state)
        data['timestamp'] = rospy.get_time()
        data['connection_quality'] = connection_quality 
        
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
