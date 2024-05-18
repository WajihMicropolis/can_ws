#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from std_msgs.msg import Float32, Int16MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu
from json_data import data

PI = 3.14159
wheel_radius = 0.24 # in meters

class RobotFeedback:
    def __init__(self):
        self.motors_speed_sub           = rospy.Subscriber("motors_speed",   Int16MultiArray, self.motors_speed_cb)
        self.steering_angle_sub         = rospy.Subscriber("steering_angle",     Int16MultiArray, self.steering_angle_cb)
        self.brake_percentage_sub       = rospy.Subscriber("brake_percentage",   Int16MultiArray, self.steering_angle_cb)
        self.ultrasonic_sub             = rospy.Subscriber("ultrasonic",     Int16MultiArray, self.ultrasonic_cb)
        self.rpy_sub                    = rospy.Subscriber("rpy",     Int16MultiArray, self.rpy_cb)

        self.battery_sub                = rospy.Subscriber("battery",    BatteryState, self.battery_cb)
        self.robot_speed_sub            = rospy.Subscriber("robot_speed",    Float32, self.robot_speed_cb)
        self.door_state_sub             = rospy.Subscriber("door_state",     String, self.door_state_cb)
        self.steering_status_sub        = rospy.Subscriber("steering_status",    String, self.steering_status_cb)
        self.brake_status_sub           = rospy.Subscriber("brake_status",   String, self.brake_status_cb)
        self.imu_sub                    = rospy.Subscriber("imu",    Imu, self.imu_cb)
        self.steering_health_check_sub  = rospy.Subscriber("steering_health_check",  Bool, self.steering_health_check_cb)
        self.braking_health_check_sub   = rospy.Subscriber("braking_health_check",   Bool, self.braking_health_check_cb)


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

    def steering_angle_cb(self, msg: Int16MultiArray):
        data['motors_details']['fr']['brake'] = msg.data[0]
        data['motors_details']['fl']['brake'] = msg.data[1]
        data['motors_details']['br']['brake'] = msg.data[2]
        data['motors_details']['bl']['brake'] = msg.data[3]

    def ultrasonic_cb(self, msg):
        None
    
    def rpy_cb(self, msg: Int16MultiArray):
        data['orientation']['roll'] = msg.data[0]
        data['orientation']['pitch'] = msg.data[1]
        data['orientation']['yaw']  = msg.data[2]

    def battery_cb(self, msg: BatteryState):
        data['battery']['percentage'] = msg.percentage

    def robot_speed_cb(self, msg:Float32):
        speed_rpm = msg.data
        speed_kmh = (speed_rpm * 2 * PI * wheel_radius * 60) / 1000
        data['speed'] = int (speed_kmh)

    def door_state_cb(self, msg:String):
        data['door_state'] = msg.data
        print("door state: ", data['door_state'])

    def steering_status_cb(self, msg:String):
        data['steering_status'] = msg.data
        None
    def brake_status_cb(self, msg:String):
        data['brake_status'] = msg.data
        None
    def imu_cb(self, msg):
        None
    def steering_health_check_cb(self, msg):
        None
    def braking_health_check_cb(self, msg):
        None

if __name__ == "__main__":
    try:
        robot = RobotFeedback()

        while not rospy.is_shutdown():
            # robot.priority_control()
            rospy.sleep(0.03)

    except rospy.ROSInterruptException:
        pass
