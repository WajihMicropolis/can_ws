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
        self.motors_speed_sub           = rospy.Subscriber("feedback/motors_speed",   Int16MultiArray, self.motors_speed_cb)
        self.steering_angle_sub         = rospy.Subscriber("feedback/steering_angle",     Int16MultiArray, self.steering_angle_cb)
        self.brake_percentage_sub       = rospy.Subscriber("feedback/brake_percentage",   Int16MultiArray, self.brake_percentage_cb)
        self.ultrasonic_sub             = rospy.Subscriber("feedback/ultrasonic",     Int16MultiArray, self.ultrasonic_cb)
        self.rpy_sub                    = rospy.Subscriber("feedback/rpy",     Int16MultiArray, self.rpy_cb)

        self.battery_sub                = rospy.Subscriber("feedback/battery",    BatteryState, self.battery_cb)
        # self.imu_sub                    = rospy.Subscriber("feedback/imu",    Imu, self.imu_cb)
        self.robot_speed_sub            = rospy.Subscriber("feedback/robot_speed",    Float32, self.robot_speed_cb)
        self.door_state_sub             = rospy.Subscriber("feedback/door_state",     String, self.door_state_cb)
        self.steering_state_sub         = rospy.Subscriber("feedback/steering_state",    String, self.steering_state_cb)
        self.brake_state_sub            = rospy.Subscriber("feedback/brake_state",   String, self.brake_state_cb)
        self.driving_mode_sub           = rospy.Subscriber("feedback/driving_mode",    String, self.driving_mode_cb)

        self.steering_health_check_sub  = rospy.Subscriber("feedback/steering_health_check",  Bool, self.steering_health_check_cb)
        self.braking_health_check_sub   = rospy.Subscriber("feedback/braking_health_check",   Bool, self.braking_health_check_cb)


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

    def steering_state_cb(self, msg:String):
        
        data['steering_status'] = msg.data
            
    def brake_state_cb(self, msg:String):
        data['brake_status'] = msg.data
        None

    def driving_mode_cb(self, msg:String):
        data['drivingMode'] = msg.data
        None

    def steering_health_check_cb(self, msg:Bool):
        None
    def braking_health_check_cb(self, msg: Bool):
        None

if __name__ == "__main__":
    try:
        robot = RobotFeedback()
        test_data = "ok:MotorOverTemperature:MotorOverCurrent:MotorError"
        # split test_data by ':'
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
