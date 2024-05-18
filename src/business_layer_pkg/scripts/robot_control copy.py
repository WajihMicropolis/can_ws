#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from std_msgs.msg import Float32, Int16MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu
# from .dummy import data
from robot_feedback import *



class robotNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("robot_control", anonymous=True)

        self.gear_sub = rospy.Subscriber("gear", String, self.gear_cb)
        self.wasdb_sub = rospy.Subscriber("wasdb", String, self.wasdb_cb)
        
        self.tele_operator_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1, latch=True)
        self.velocity_pub = rospy.Publisher("robot/velocity",Float32,queue_size=1 ,latch=True)
        self.steering_pub = rospy.Publisher("robot/steering",Float32,queue_size=1,latch=True)
        self.emergency_brake_pub = rospy.Publisher("robot/emergency_brake",Bool,queue_size=1,latch=True)

        

        self.prev_drive_timer = rospy.get_time()
        self.prev_steer_timer = rospy.get_time()

        self.navigation_command = Twist()
        self.teleoperator_command = Twist()
        self.cmd_vel_msg = Twist()
        
        self.robot_velocity_rpm = Float32()
        self.robot_steering = Float32()
        self.robot_emergency_brake = Bool()
        
        
        self.steering_angle_as_rad = False

        self.gear = 2
        self.drive_data = {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}
        self.robot_speed = 0
        self.robot_steering_angle = 0

        self.drive_change_time = 0.3
        self.steer_change_time = 0.075

        self.speed_step = 0.25
        self.robot_max_forward_speed = self.speed_step * self.gear +0.1
        self.robot_max_reverse_speed = -0.50
        self.robot_speed_change_rate = 0.1
        self.speed_goal = 0

        self.brake = True
        self.prev_brake = False
        self._priority_control = False

        self.robot_max_steering_angle = 16.0 if not self.steering_angle_as_rad else 0.3
        self.robot_min_steering_angle = (
            -16.0 if not self.steering_angle_as_rad else -0.3
        )
        self.robot_steering_angle_change_rate = (
            1.0 if not self.steering_angle_as_rad else 0.05
        )
        self.steering_goal = 0


    
    def gear_cb(self, msg: String):

        gear = json.loads(msg.data)
        self.gear = gear["gear"]
        print("gear: ", type(self.gear))
        self.robot_max_forward_speed = self.speed_step * self.gear +0.1
        print("robot max speed: ", self.robot_max_forward_speed)

    def wasdb_cb(self, msg: String):

        wasdb = json.loads(msg.data)
        wasdb = wasdb["wasdb"]
        # print(wasdb)
        self.drive_data["w"] = wasdb["w"]
        self.drive_data["a"] = wasdb["a"]
        self.drive_data["s"] = wasdb["s"]
        self.drive_data["d"] = wasdb["d"]
        self.drive_data["b"] = wasdb["b"]
        if self.drive_data["b"] and not self.prev_brake:
            self.brake = True

        self.prev_brake = self.drive_data["b"]
        self.update_speed_goal()

    def update_speed_goal(self):
        if self.drive_data["b"] == 1:
            print("brake")
            self.speed_goal = 0
            self.robot_emergency_brake.data = True


        elif self.drive_data["w"] == 1 and self.drive_data["s"] == 0:
            self.speed_goal = self.robot_max_forward_speed 
            self.robot_speed_change_rate = 0.1
            self.robot_emergency_brake.data = False


        elif self.drive_data["w"] == 0 and self.drive_data["s"] == 1:
            self.speed_goal = self.robot_max_reverse_speed
            self.robot_speed_change_rate = 0.1
            self.robot_emergency_brake.data = False

        elif self.drive_data["w"] == 0 and self.drive_data["s"] == 0 and self.drive_data["b"] == 0:
            self.speed_goal = 0
            self.robot_emergency_brake.data = False

        self.robot_speed_change_rate = round(self.robot_speed_change_rate, 3)
        self.speed_goal = round(self.speed_goal, 3)

    def update_steering_angle_goal(self):
        if self.drive_data["a"] == 1 and self.drive_data["d"] == 0:
            self.steering_goal = self.robot_max_steering_angle

        elif self.drive_data["a"] == 0 and self.drive_data["d"] == 1:
            self.steering_goal = self.robot_min_steering_angle

        elif self.drive_data["a"] == 0 and self.drive_data["d"] == 0:
            self.steering_goal = self.robot_steering_angle

    def drive_control(self):

        self.update_speed_goal()

        if self.speed_goal == 0:
            self.robot_speed = 0
            return
        
        self.robot_speed = self.speed_goal

        self.robot_speed = round(self.robot_speed, 3)
        self.robot_speed = min(
            self.robot_max_forward_speed,
            max(self.robot_max_reverse_speed, self.robot_speed),
        )

    def steering_control(self):
        if rospy.get_time() - self.prev_steer_timer < self.steer_change_time:
            return
        self.prev_steer_timer = rospy.get_time()
        self.update_steering_angle_goal()

        if self.robot_steering_angle < self.steering_goal:
            self.robot_steering_angle += self.robot_steering_angle_change_rate
        elif self.robot_steering_angle > self.steering_goal:
            self.robot_steering_angle -= self.robot_steering_angle_change_rate
        else:
            self.robot_steering_angle = self.steering_goal

        self.robot_steering_angle = round(self.robot_steering_angle, 2)

    def keyboard_control(self, _print=False):
        self.drive_control()
        self.steering_control()

        self.teleoperator_command.linear.x = self.robot_speed
        self.teleoperator_command.angular.z = self.robot_steering_angle
        self.tele_operator_pub.publish(self.teleoperator_command)
        
        # convert linear.x from m/s to rpm (maximum 100 rpm)
        
        # rpm * 2 * pi *r / 60 = m/s
        self.robot_velocity_rpm = self.robot_speed* 60 / (2 * PI * wheel_radius)
        self.robot_steering = self.robot_steering_angle
        
        self.velocity_pub.publish(self.robot_velocity_rpm)
        self.steering_pub.publish(self.robot_steering)
        self.emergency_brake_pub.publish(self.robot_emergency_brake)
        
        if not _print:
            return
        print("robot speed: ", self.robot_speed)
        print("speed goal: ", self.speed_goal)
        print("robot steering angle: ", self.robot_steering_angle)
        print("robot_speed_change_rate", self.robot_speed_change_rate)
        print("robot gear ", self.gear)
        print("brake: ", self.brake)
        # print("steering goal: ", self.steering_goal)
        print("---------------------")

        
    # def priority_control(self):

    #     if self.robot_steering_angle != 0.0:
    #         self.cmd_vel_msg.angular = self.teleoperator_command.angular
    #     else:
    #         self.cmd_vel_msg.angular = self.navigation_command.angular

    #     if self.drive_data["w"] or self.drive_data["s"] or self.drive_data["b"] :
    #         self.cmd_vel_msg.linear = self.teleoperator_command.linear
    #     else:
    #         self.cmd_vel_msg.linear = self.navigation_command.linear

    #     print("linear.x: ", self.cmd_vel_msg.linear.x)
    #     self.cmd_vel_pub.publish(self.cmd_vel_msg)


if __name__ == "__main__":
    try:
        robot = robotNode()

        while not rospy.is_shutdown():
            robot.keyboard_control(_print=True)
            # robot.priority_control()
            rospy.sleep(0.03)

    except rospy.ROSInterruptException:
        pass
