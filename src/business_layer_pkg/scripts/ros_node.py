#!/usr/bin/env python3
import rospy
import json
from copy import deepcopy

from std_msgs.msg import String
from std_msgs.msg import Float32, Int16MultiArray, Bool
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

        #! publishers to the Teleoperation software
        self._robot_state_pub               = rospy.Publisher("robot_state", String, queue_size=1, latch=True)
        self._health_check_pub              = rospy.Publisher("health_check", String, queue_size=1, latch=True)
        self._emetgency_cause_pub           = rospy.Publisher("emergency_cause", String, queue_size=1, latch=True)
        self._robot_operational_details_pub = rospy.Publisher("robot_operational_details", String, queue_size=1, latch=True)
        
        #! publishers to the Robot
        self.tele_operator_pub      = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=True)
        self.velocity_pub           = rospy.Publisher("robot/velocity",Float32,queue_size=1 ,latch=True)
        self.steering_pub           = rospy.Publisher("robot/steering",Float32,queue_size=1,latch=True)
        self.emergency_brake_pub    = rospy.Publisher("robot/emergency_brake",Bool,queue_size=1,latch=True)
        self.robot_door_control_pub = rospy.Publisher("robot/door_control",Bool,queue_size=1,latch=True)

        #! subscribers from the Teleoperation software
        self.gear_sub           = rospy.Subscriber("gear", String, self.gear_cb)
        self.wasdb_sub          = rospy.Subscriber("wasdb", String, self.wasdb_cb)
        self.door_control_sub   = rospy.Subscriber("door_control", String, self.door_control_cb)

        # services
        self._health_check_srv = rospy.Service("health_check_srv", health_check, self.health_check_srv)

        self.teleoperator_command = Twist()
        
        self.robot_velocity_rpm         = Float32()
        self.robot_steering             = Float32()
        self.robot_emergency_brake      = Bool()
        self.robot_operational_details  = String()

        self.robot_state = "KEY_OFF"
        self.prev_robot_state = ""
        self.old_robot_state = ""

        self.connection_quality = 75
        self.steering_health_check = True
        self.braking_health_check = True
        self.battery_capacity = 100

        self._modeToBeChecked = []
        self.gear = 1

        self.ip = "10.20.0.30"
        self.Robot_Control = Robot_Control()
        self.Robot_Feedback = RobotFeedback(self.ip)

        self.drive_data = {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}

        self.check_index, self.status_index = 0 , 0
        self.check_time = 0.3
        self.prev_health_check_time = rospy.Time.now()

        self.emergency_cause,  self.prev_emergency_cause = "", ""
        self.emergency_check_time = 0.3
        self.prev_emergency_cause_time = rospy.Time.now()

        self.init_time = rospy.Time.now().to_sec()
        self.elapsed_time = "00:00:00"

    def health_check_srv(self, req: health_checkRequest):
        next_mode = req.nextMode

        if next_mode not in _nextModeArray or next_mode == self.robot_state:
            rospy.logerr(f"Invalid or Repeated Mode: {next_mode}")
            return health_checkResponse(checks=[])

        rospy.logdebug(f"health_check_srv: {next_mode}")
        self._modeToBeChecked = []

        self.robot_state = next_mode
        if next_mode == "STAND_BY":
            self._modeToBeChecked.append(_healthCheckArray[0])

        elif next_mode == "FREE_DRIVING":
            for i in range(3):
                self._modeToBeChecked.append(_healthCheckArray[i])

        elif next_mode == "MAPPING":
            self._modeToBeChecked.append(_healthCheckArray[3])

        elif next_mode == "MISSION_INDOOR":
            self._modeToBeChecked.append(_healthCheckArray[4])

        elif next_mode == "MISSION_OUTDOOR":
            self._modeToBeChecked.append(_healthCheckArray[4])

        print("mode to be checked: ", self._modeToBeChecked)
        return health_checkResponse(checks=self._modeToBeChecked)

    def door_control_cb(self, msg: String):
        door_control = json.loads(msg.data)
        print("door_control: ", door_control)

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
                    status = _modeCheckStatus[self.status_index]
                
                # print(f"{check}:{status}")
                self._health_check_pub.publish(f"{check}:{status}")
                self.status_index += 1
            else:
                self.status_index = 0
                self.check_index += 1
        else:
            self.check_index = 0
            self.status_index = 0
            self._modeToBeChecked = []

        
    def publish_emergency_cause(self):
        if rospy.Time.now() - self.prev_emergency_cause_time < rospy.Duration(self.emergency_check_time):
            return
        self.prev_emergency_cause_time = rospy.Time.now()

        self.emergency_cause = self.Robot_Feedback.getEmergencyCause()

        if self.emergency_cause != self.prev_emergency_cause:
            
            self.robot_state = self.old_robot_state if self.emergency_cause == "" else "EMERGENCY"

            self.prev_emergency_cause = deepcopy(self.emergency_cause)
            self._emetgency_cause_pub.publish(self.emergency_cause)
            # print("emergency_cause: ", self.emergency_cause)

    def publish_robot_state(self):

        if self.robot_state != self.prev_robot_state:
            if self.robot_state != "EMERGENCY":
                self.old_robot_state = self.robot_state

            if self.robot_state != "KEY_OFF": # reset the timer
                self.init_time = rospy.Time.now().to_sec()

            print("robot_state: ", self.robot_state)
            self.prev_robot_state = deepcopy(self.robot_state)
            self._robot_state_pub.publish(self.robot_state)

    def publish_teleop(self):
        self.Robot_Control.teleop_control(self.teleoperator_command, self.robot_velocity_rpm, self.robot_steering, self.robot_emergency_brake)
        self.tele_operator_pub.publish(self.teleoperator_command)
        self.velocity_pub.publish(self.robot_velocity_rpm)
        self.steering_pub.publish(self.robot_steering)
        self.emergency_brake_pub.publish(self.robot_emergency_brake)
        
    def publish_robot_operational_details(self):
        self.elapsed_time = self.update_elapsed_time()
        self.robot_operational_details = self.Robot_Feedback.getRobotFeedback(self.elapsed_time)
        self._robot_operational_details_pub.publish(self.robot_operational_details)
        
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

    def update(self):

        self.publish_robot_state()
        self.publish_health_check()
        self.publish_emergency_cause()
        self.publish_robot_operational_details()

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