#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import Float32, Int8, Bool
from geometry_msgs.msg import Twist
from robot_feedback import *



class Robot_Control:
    def __init__(self):

        self.prev_steer_timer = rospy.get_time()

        self.steering_angle_as_rad  = False
        self.robot_velocity_rpm     = Float32()
        self.robot_steering         = Float32()
        self.robot_emergency_brake  = Bool()
        self.robot_door_control     = Int8()
        self.teleoperator_command   = Twist()
        self.robot_command          = Twist()
        
        self.overTakeTimeDrive = rospy.get_time()
        self.overTakeTimeSteering = rospy.get_time()

        self.gear = 1
        self.drive_data = {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}
        self.robot_speed = 0
        self.robot_steering_angle = 0

        self.steer_change_time = 0.06

        self.speed_step = 0.5
        self.robot_max_forward_speed = self.speed_step * self.gear +0.1
        self.robot_max_reverse_speed = -1.0
        self.speed_goal = 0

        self.brake = True
        self.prev_brake = False
        self._priority_control = False

        self.robot_max_steering_angle = 16.0 if not self.steering_angle_as_rad else 0.3
        self.robot_min_steering_angle = (
            -16.0 if not self.steering_angle_as_rad else -0.3
        )
        self.robot_steering_angle_change_rate = (1.0 if not self.steering_angle_as_rad else 0.05)
        self.steering_goal = 0


    
    def gear_update(self, gear):
        self.gear = gear
        self.robot_max_forward_speed = self.speed_step * self.gear +0.1
        print("gear: ", self.gear)
        print("robot max speed: ", self.robot_max_forward_speed)

    def wasdb_update(self, wasdb):

        # print(wasdb)
        self.drive_data["w"] = wasdb["w"]
        self.drive_data["a"] = wasdb["a"]
        self.drive_data["s"] = wasdb["s"]
        self.drive_data["d"] = wasdb["d"]
        self.drive_data["b"] = wasdb["b"]
        if self.drive_data["b"] and not self.prev_brake:
            self.brake = True

        self.prev_brake = self.drive_data["b"]
        self.update_speed()

    def update_speed(self):
        if self.drive_data["b"] == 1:
            # print("brake")
            self.speed_goal = 0
            self.robot_emergency_brake.data = True


        elif self.drive_data["w"] == 1 and self.drive_data["s"] == 0:
            self.speed_goal = self.robot_max_forward_speed 
            self.robot_emergency_brake.data = False


        elif self.drive_data["w"] == 0 and self.drive_data["s"] == 1:
            self.speed_goal = self.robot_max_reverse_speed
            self.robot_emergency_brake.data = False

        elif self.drive_data["w"] == 0 and self.drive_data["s"] == 0 and self.drive_data["b"] == 0:
            self.speed_goal = 0
            self.robot_emergency_brake.data = False

        self.speed_goal = round(self.speed_goal, 3)

        if self.speed_goal == 0:
            self.robot_speed = 0
            return
        
        self.robot_speed = self.speed_goal

        self.robot_speed = round(self.robot_speed, 3)
        self.robot_speed = min(
            self.robot_max_forward_speed,
            max(self.robot_max_reverse_speed, self.robot_speed),
        )

    def update_steering_angle_goal(self):
        if self.drive_data["a"] == 1 and self.drive_data["d"] == 0:
            self.steering_goal = self.robot_max_steering_angle

        elif self.drive_data["a"] == 0 and self.drive_data["d"] == 1:
            self.steering_goal = self.robot_min_steering_angle

        elif self.drive_data["a"] == 0 and self.drive_data["d"] == 0:
            self.steering_goal = self.robot_steering_angle
        

    def update_steering(self):
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

    def priority_control(self, auto_pilot_command = Twist(), teleoperator_command = Twist()):
        
        auto_velocity, auto_steering = auto_pilot_command.linear.x, auto_pilot_command.angular.z
        teleop_velocity, teleop_steering = teleoperator_command.linear.x, teleoperator_command.angular.z
        
        if  self.drive_data["w"] or self.drive_data["s"] or self.drive_data["b"]:
            self.robot_command.linear.x = teleop_velocity
            self.robot_command.angular.z = teleop_steering        
            self.overTakeTimeDrive = rospy.get_time()
            
        else:
            if rospy.get_time() - self.overTakeTimeDrive > 3: 
                self.robot_command.linear.x = auto_velocity
            else: #! The robot will stop if no command is received from the teleoperator for 3 seconds
                self.robot_command.linear.x = 0
        
        if self.drive_data["a"] or self.drive_data["d"]:
            self.robot_command.angular.z = teleop_steering
            self.overTakeTimeSteering = rospy.get_time()
        else:
            if rospy.get_time() - self.overTakeTimeSteering > 3:
                if auto_steering != 0.0:
                    self.robot_steering_angle = 0
                    self.robot_command.angular.z = auto_steering
                else:
                    self.robot_command.angular.z = teleop_steering
        # print("teleop_steering: ", teleop_steering)
        # print("angular.z: ", self.robot_command.angular.z)
            
        
        return self.robot_command
    
    def teleop_control(self):
        self.update_speed()
        self.update_steering()

        self.teleoperator_command.linear.x = self.robot_speed
        self.teleoperator_command.angular.z = self.robot_steering_angle
        
        return self.teleoperator_command
    
    def get_robot_command(self,cmd_vel = Twist() ,velocity_rpm = Float32(),steering = Float32(),emergency_brake = Bool()):
        
        # convert linear.x from m/s to rpm (maximum 100 rpm)
        # rpm * 2 * pi *r / 60 = m/s
        velocity_rpm.data = cmd_vel.linear.x* 60 / (2 * PI * wheel_radius)
        steering.data = cmd_vel.angular.z
        emergency_brake.data = self.robot_emergency_brake.data


if __name__ == "__main__":
    try:
        robot = Robot_Control()

        while not rospy.is_shutdown():
            robot.teleop_control()
            rospy.sleep(0.03)

    except rospy.ROSInterruptException:
        pass
