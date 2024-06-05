#!/usr/bin/env python3
from sympy import capture
import rospy
import json
import os
from copy import deepcopy
import subprocess

from std_srvs.srv import Trigger, TriggerResponse
from business_layer_pkg.srv import end_map,end_mapRequest ,end_mapResponse

from std_msgs.msg import String
from std_msgs.msg import Float32, Int8MultiArray, Bool, Int8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu
from business_layer_pkg.srv import (
    health_check,
    health_checkRequest,
    health_checkResponse,
)
from robot_control import Robot_Control
from robot_feedback import RobotFeedback
import datetime
import requests

_healthCheckArray = [
    "CONNECTION_QUALITY",
    "HARDWARE",
    "BATTERY_LEVEL",
    "SENSOR",
    "AUTOPILOT",
]
_nextModeArray = [
    "STAND_BY",
    "FREE_DRIVING",
    "MAPPING",
    "MISSION_INDOOR",
    "MISSION_OUTDOOR",
]

_modeCheckStatus = ["STARTED", "PENDING", "SUCCEEDED", "FAILED"]


class Robot_Node:
    def __init__(self):
        rospy.init_node("robot_node", anonymous=True)
        self.ip = self.get_public_ip()
        
        #! publishers to the Teleoperation software
        self._robot_state_pub               = rospy.Publisher("robot_state", String, queue_size=1, latch=True)
        self._health_check_pub              = rospy.Publisher("health_check", String, queue_size=1, latch=True)
        self._emetgency_cause_pub           = rospy.Publisher("emergency_cause", String, queue_size=1, latch=True)
        self._robot_operational_details_pub = rospy.Publisher("robot_operational_details", String, queue_size=1, latch=True)
        #! subscribers from the Teleoperation software
        self._gear_sub           = rospy.Subscriber("gear", String, self.gear_cb)
        self._wasdb_sub          = rospy.Subscriber("wasdb", String, self.wasdb_cb)
        self._door_control_sub   = rospy.Subscriber("door_control", String, self.door_control_cb)
        
        #! publishers to the Robot
        self.tele_operator_pub      = rospy.Publisher("teleop_cmd_vel", Twist, queue_size=1, latch=True)
        self.velocity_pub           = rospy.Publisher("robot/velocity",Float32,queue_size=1 ,latch=True)
        self.steering_pub           = rospy.Publisher("robot/steering",Float32,queue_size=1,latch=True)
        self.emergency_brake_pub    = rospy.Publisher("robot/emergency_brake",Bool,queue_size=1,latch=True)
        self.robot_door_control_pub = rospy.Publisher("robot/door_control",Int8MultiArray,queue_size=1,latch=True)
        #! subscribers from auto-pilot
        self._auto_pilot_sub = rospy.Subscriber("autonomous_cmd_vel", Twist, self.autonomous_cmd_vel_cb)

        # services
        self._health_check_srv  = rospy.Service("health_check_srv", health_check, self.health_check_srv)
        self._streams_srv        = rospy.Service("streams_ids", Trigger, self.req_streames_srv)
        self._start_map_srv      = rospy.Service("start_map_srv", Trigger, self.start_map_srv)
        self._end_map_srv        = rospy.Service("end_map_srv", end_map, self.end_map_srv)

        #! variables
        self.teleoperator_command   = Twist()
        self.auto_pilot_command     = Twist()
        self.robot_command          = Twist()
        
        self.robot_velocity_rpm             = Float32()
        self.robot_steering                 = Float32()
        self.robot_emergency_brake          = Bool()
        self.robot_operational_details      = String()
        self.prev_robot_operational_details = String()
        self.door_control                   = Int8MultiArray()
        self.prev_door_control              = Int8MultiArray()
        

        self.robot_state = "STAND_BY"
        self.next_mode   = "STAND_BY"
        self.prev_robot_state = ""
        self.old_robot_state = ""

        self.connection_quality = 75
        self.steering_health_check = True
        self.braking_health_check = True
        self.battery_capacity = 100
        self.health_check_success = 0

        self._modeToBeChecked = []
        self.gear = 1
        self.door_control.data = [1,1,1,1,1]
        
        self.Robot_Control = Robot_Control()
        self.Robot_Feedback = RobotFeedback(self.ip)

        self.drive_data = {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}

        self.check_index, self.status_index = 0 , 0
        self.check_time = 0.3
        self.prev_health_check_time = rospy.Time.now()

        self.emergency_cause = self.Robot_Feedback.getEmergencyCause()
        self.prev_emergency_cause = deepcopy(self.emergency_cause)
        self.emergency_cause_ok = deepcopy(self.emergency_cause)
        
        
        self.emergency_check_time = 0.3
        self.prev_emergency_cause_time = rospy.Time.now()

        self.init_time = rospy.Time.now().to_sec()
        self.elapsed_time = "00:00:00"
        self.prev_publish_time = rospy.Time.now()
        
        self.start_mapping = False
        
        self.pod_doors_control = {
            "top":0,
            "front_right": 0,
            "front_left": 0,
            "back_right": 0,
            "back_left": 0
        }
        print("Robot Node is ready, ", rospy.Time.now().to_sec())

    def health_check_srv(self, req: health_checkRequest):
        self.next_mode = req.nextMode
        
        #! or self.next_mode == self.robot_state
        if self.next_mode not in _nextModeArray :
            rospy.logerr(f"Invalid : {self.next_mode}")
            return health_checkResponse(checks=[])

        rospy.logdebug(f"health_check_srv: {self.next_mode}")
        self._modeToBeChecked.clear()

        # self.robot_state = self.next_mode
        if self.next_mode == "STAND_BY":
            self._modeToBeChecked.append(_healthCheckArray[0])

        elif self.next_mode == "FREE_DRIVING":
            for i in range(3):
                self._modeToBeChecked.append(_healthCheckArray[i])

        elif self.next_mode == "MAPPING":
            self._modeToBeChecked.append(_healthCheckArray[3])
            # todo : add check for server connection to save map
        elif self.next_mode == "MISSION_INDOOR":
            self._modeToBeChecked.append(_healthCheckArray[4])

        elif self.next_mode == "MISSION_OUTDOOR":
            self._modeToBeChecked.append(_healthCheckArray[4])

        print("mode to be checked: ", self._modeToBeChecked)
        return health_checkResponse(checks=self._modeToBeChecked)

    def get_public_ip(self):
        try:
            response = requests.get('https://api.ipify.org?format=json')
            response.raise_for_status()  # Raise an exception for HTTP errors
            ip_info = response.json()
            return ip_info['ip']
        except requests.RequestException as e:
            print(f"Error occurred: {e}")
            return None
    
    def req_streames_srv(self,req):
        public_ip = self.ip
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

    def start_map_srv(self,req):
        package = 'octomap_server'
        launch_file = 'octomap_mapping.launch'
        args = 'open_rviz:=false'
        ros_env = "~/webots_dir/devel/setup.bash"
        
        command = "roslaunch {0} {1} {2}".format(package, launch_file, args)

        result = subprocess.Popen(command)
        state = result.poll()


        print("state: ", state)
        print("stdout: ", result.stdout)
        print("stderr: ", result.stderr)
        
        if state is None:
            rospy.loginfo("process is running fine")
        elif state < 0:
            rospy.loginfo("Process terminated with error")
        elif state > 0:
            rospy.loginfo("Process terminated without error")
        print("start_map_srv")
        self.start_mapping = True
        return TriggerResponse(success=True, message="map started")

    def end_map_srv(self,req: end_mapRequest):
        if not self.start_mapping:
            return end_mapResponse(success=False)
        print("req save map: ",req.save_map)
        print("req map name: ",req.map_name)
        # todo : save the map and send it to the server
        
        if req.save_map:
            print("save map")
            
            command = f"bash -c ' rosrun map_server map_saver -f /home/microspot/catkin_ws/maps/{req.map_name} map:=/projected_map'"
            result = subprocess.run(command, capture_output=True, text=True, shell=True,timeout=3)
            if 'Done' in result.stdout:
                print("Map saved successfully.")
            else:
                print("Map saving failed. Check the output for details.")
                print("stdout:", result.stdout)
                print("stderr:", result.stderr)
                return end_mapResponse(success=False, status_message="map not saved")
        
        
        kill_command = "rosnode kill /octomap_server"
        p = subprocess.Popen(kill_command, shell=True)

        state = p.poll()
        if state is None:
            rospy.loginfo("process is running fine")
        elif state < 0:
            rospy.loginfo("Process terminated with error")
        elif state > 0:
            rospy.loginfo("Process terminated without error")
        return end_mapResponse(success=True, status_message="map ended")
    
    def map_value(self,x, a, b, c, d):
        """
        Maps a value x from range [a, b] to range [c, d].

        Parameters:
        - x: The value to map.
        - a: The lower bound of the original range.
        - b: The upper bound of the original range.
        - c: The lower bound of the new range.
        - d: The upper bound of the new range.

        Returns:
        - The value of x mapped to the new range [c, d].
        """
        return float(c + (x - a) * (d - c) / (b - a))

    def autonomous_cmd_vel_cb(self, msg: Twist):
        self.auto_pilot_command.linear = msg.linear
        # map the steering angle from rad to degree
        self.auto_pilot_command.angular.z = self.map_value(msg.angular.z, -0.25, 0.25, -16, 16)
        self.auto_pilot_command.angular.z = min(16,max(-16,self.auto_pilot_command.angular.z))
        
    def door_control_cb(self, msg: String):
        if not msg.data:
            return
        
        if not hasattr(self, 'pod_doors_control'):
            rospy.logerr("Variable 'pod_doors_control' is not defined")
            return

        _door_control = json.loads(msg.data)
        _target_door = _door_control["target_door"]
        _target_state = _door_control["target_state"]
        
        # print("target_door: ", _target_door)
        # print("target_state: ",_target_state)
        self.pod_doors_control[_target_door] = 0 if _target_state =="open" else 2
        # print(self.pod_doors_control)
        self.door_control.data = [self.pod_doors_control["top"] ,self.pod_doors_control["front_right"], self.pod_doors_control["front_left"], self.pod_doors_control["back_right"], self.pod_doors_control["back_left"]]

    def gear_cb(self, msg: String):

        gear = json.loads(msg.data)
        self.gear = gear["gear"]
        self.Robot_Control.gear_update(self.gear)

    def wasdb_cb(self, msg: String):

        wasdb = json.loads(msg.data)
        wasdb = wasdb["wasdb"]
        self.drive_data = wasdb
        self.Robot_Control.wasdb_update(self.drive_data)
    
    
    def publish_health_check(self):

        self.steering_health_check, self.braking_health_check = self.Robot_Feedback.getHealthCheck()
        
        if self._modeToBeChecked == []:
            return

        if rospy.Time.now() - self.prev_health_check_time < rospy.Duration(self.check_time):
            return
        self.prev_health_check_time = rospy.Time.now()

        status_checks = {
            "CONNECTION_QUALITY": lambda: self.connection_quality >= 70,
            "HARDWARE": lambda: self.steering_health_check
            and self.braking_health_check,
            "BATTERY_LEVEL": lambda: self.battery_capacity > 30,
        }
        if self.check_index < len(self._modeToBeChecked):
            check = self._modeToBeChecked[self.check_index]
            if self.status_index < (len(_modeCheckStatus) -1):

                if self.status_index == 2:
                    status = (_modeCheckStatus[2]if status_checks[check]()else _modeCheckStatus[3])

                    if status == _modeCheckStatus[3]:
                        error_messages = {
                            "CONNECTION_QUALITY": "connection error",
                            "HARDWARE": "steering error" if not self.steering_health_check else "brake error" if not self.braking_health_check else "hardware error",
                            "BATTERY_LEVEL": "low battery",
                        }
                        status += f":{error_messages[check]}"
                    else:
                        self.health_check_success += 1
                else:
                    status = _modeCheckStatus[self.status_index]
                
                self._health_check_pub.publish(f"{check}:{status}")
                self.status_index += 1
            else:
                self.status_index = 0
                self.check_index += 1
                
        else:
            if self.health_check_success == len(self._modeToBeChecked):
                print("health check success")
                self.robot_state = self.next_mode
            self.check_index = 0
            self.status_index = 0
            self.health_check_success = 0
            self._modeToBeChecked = []
        
            
    def publish_doors_control(self):
        
        if self.door_control.data != self.prev_door_control.data:
            self.prev_door_control.data = deepcopy(self.door_control.data)
            self.robot_door_control_pub.publish(self.door_control)

        
    def publish_emergency_cause(self):
        if rospy.Time.now() - self.prev_emergency_cause_time < rospy.Duration(self.emergency_check_time):
            return
        self.prev_emergency_cause_time = rospy.Time.now()

        self.emergency_cause = self.Robot_Feedback.getEmergencyCause()
        # print("emergency_cause: ", self.emergency_cause)
        # print("prev_emergency_cause: ", self.prev_emergency_cause)
        if self.emergency_cause != self.prev_emergency_cause:
            # todo in case of no emergency make it STAND_BY
            self.robot_state = self.old_robot_state if self.emergency_cause == self.emergency_cause_ok else "EMERGENCY"
            self.prev_emergency_cause = deepcopy(self.emergency_cause)
            
            if self.emergency_cause != self.emergency_cause_ok:
                self._emetgency_cause_pub.publish(self.emergency_cause)
            # print("emergency_cause: ", self.emergency_cause)

    def publish_robot_state(self):

        if self.robot_state != self.prev_robot_state:
            if self.robot_state != "EMERGENCY":
                self.old_robot_state = self.robot_state

            if self.robot_state != "KEY_OFF": # reset the timer
                self.init_time = rospy.Time.now().to_sec()

            print("PUBLISH robot_state: ", self.robot_state)
            self.prev_robot_state = deepcopy(self.robot_state)
            self._robot_state_pub.publish(self.robot_state)

    def publish_teleop(self):
        self.teleoperator_command = self.Robot_Control.teleop_control()
        self.robot_command = self.Robot_Control.priority_control(self.auto_pilot_command, self.teleoperator_command )
        
        self.Robot_Control.get_robot_command(self.robot_command, self.robot_velocity_rpm, self.robot_steering, self.robot_emergency_brake)
        
        self.robot_emergency_brake.data = True if self.connection_quality < 70 else self.robot_emergency_brake.data
        
        self.tele_operator_pub.     publish(self.teleoperator_command)
        self.velocity_pub.          publish(self.robot_velocity_rpm)
        self.steering_pub.          publish(self.robot_steering)
        self.emergency_brake_pub.   publish(self.robot_emergency_brake)
        
        self.publish_doors_control()


    def update_elapsed_time(self):
        if self.robot_state == "KEY_OFF":
            return
        t = rospy.Time.now().to_sec()
        start_time = t - self.init_time
        start_time = round(start_time, 2)
        elapsed_time = datetime.timedelta(seconds=int(start_time))
        time_str = str(elapsed_time)
        # print("time: ", time_str)
        return time_str

    def publish_robot_operational_details(self):
        self.elapsed_time = self.update_elapsed_time()
        
        self.Robot_Feedback.updateDriveMode(self.robot_velocity_rpm.data)
        self.Robot_Feedback.updateDirectionLight(self.robot_steering.data)
        self.Robot_Feedback.updateDoorState()
        
        self.robot_operational_details = self.Robot_Feedback.getRobotDetails(self.elapsed_time, self.connection_quality)
        self._robot_operational_details_pub.publish(self.robot_operational_details)
        
    def update(self):

        self.publish_robot_state()
        self.publish_health_check()
        self.publish_emergency_cause()
        
        if rospy.Time.now() - self.prev_publish_time > rospy.Duration(0.2):
            self.publish_robot_operational_details()
            # self.publish_pod_details()
            self.prev_publish_time = rospy.Time.now()

        
        self.battery_capacity = self.Robot_Feedback.getBatteryCapacity()
        self.publish_teleop()

        # print("looping")
        

if __name__ == "__main__":
    try:
        robot = Robot_Node()

        while not rospy.is_shutdown():
            robot.update()
            # robot.keyboard_control(_print=True)
            # robot.priority_control()
            rospy.sleep(0.03)

    except rospy.ROSInterruptException:
        pass
