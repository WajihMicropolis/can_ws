#!/bin/python3

import rospy
import rospkg
from PyQt5 import QtCore, QtWidgets, QtGui
from std_msgs.msg import Int16MultiArray, String, Float32, Bool
from sensor_msgs.msg import BatteryState

from visualizer_V2 import Ui_Dialog  # Import the auto-generated UI class

class RobotVisualizer(QtWidgets.QDialog, Ui_Dialog):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        rospy.init_node('feedback_visualizer_node', anonymous=True)
        self.rospack = rospkg.RosPack()
        
        self.battery = BatteryState()
        
        # Subscribers
        self.motor_speed_sub        = rospy.Subscriber('feedback/motors_speed', Int16MultiArray, self.motor_speed_cb)
        self.steering_angle_sub     = rospy.Subscriber('feedback/steering_angle', Int16MultiArray, self.steering_angle_cb)
        self.brake_percentage_sub   = rospy.Subscriber('feedback/brake_percentage', Int16MultiArray, self.brake_percentage_cb)
        self.ultrasonic_sub         = rospy.Subscriber('feedback/ultrasonic', Int16MultiArray, self.ultrasonic_cb)
        self.emergency_brake_sub    = rospy.Subscriber('robot/emergency_brake', Bool, self.emergency_brake_cb)
        self.steering_state_sub     = rospy.Subscriber('feedback/steering_state', String, self.steering_state_cb)
        self.brake_state_sub        = rospy.Subscriber('feedback/brake_state', String, self.brake_state_cb)
        self.robot_velocity_sub     = rospy.Subscriber('robot/velocity', Float32, self.robot_velocity_cb)
        self.robot_steering_sub     = rospy.Subscriber('robot/steering', Float32, self.robot_steering_cb)
        self.battery_sub            = rospy.Subscriber("feedback/battery",BatteryState, lambda msg : setattr(self, 'battery', msg))

        # Variables
        self.motors_speed = Int16MultiArray()
        self.steering_angle = Int16MultiArray()
        self.brake_percentage = Int16MultiArray()
        self.ultrasonic = Int16MultiArray()
        self.steer_error = ""
        self.brake_error = ""
        self.robot_velocity = Float32()
        self.robot_steering = Float32()
        self.emergency_brake = Bool()
        
        # Image paths
        pkg_path = self.rospack.get_path('can_pkg')
        package_path = pkg_path+"/images"
        
        self.image_paths = {
            "speed_logo": package_path + "/speed_visualizer.png",
            "main_picture": package_path + "/M2_robot.png",
            "steering_logo": package_path + "/steering_visualizer.png",
            "break_logo": package_path + "/break_visualizer.png",
            "break_error_logo": package_path + "/break_error.png",
            "steering_error_logo": package_path + "/steering_error.png",
            "grey_background": package_path + "/grey_color.png",
            "FOV_up": package_path + "/FOV_up.png",
            "gear_logo": package_path + "/gear.png",
            "FOV_down": package_path + "/FOV_down.png",
            "FOV_right": package_path + "/FOV_right.png",
            "FOV_left": package_path + "/FOV_left.png",
            "battery_logo": package_path + "/battery.png"
        }
        
        #set style
        self.FR_ultrasonic_label.setStyleSheet("color: black; font-weight:bold")
        self.FL_ultrasonic_label.setStyleSheet("color: black; font-weight:bold")
        self.BR_ultrasonic_label.setStyleSheet("color: black; font-weight:bold")
        self.BL_ultrasonic_label.setStyleSheet("color: black; font-weight:bold")
        self.R_ultrasonic_label.setStyleSheet("color: black; font-weight:bold")
        self.L_ultrasonic_label.setStyleSheet("color: black; font-weight:bold")
        
        # Set images
        self.set_images()
        
        # Setup timer for updating labels
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_labels)
        self.timer.start(100)  # Update every 100 
        
        
    #*callbacks   
    def motor_speed_cb(self, msg: Int16MultiArray):
        self.motors_speed.data = msg.data

    def steering_angle_cb(self, msg: Int16MultiArray):
        self.steering_angle.data = msg.data

    def brake_percentage_cb(self, msg: Int16MultiArray):
        self.brake_percentage.data = msg.data
    
    def ultrasonic_cb(self, msg: Int16MultiArray):
        self.ultrasonic.data = msg.data

    def steering_state_cb(self, msg: String):
        self.steer_error = msg.data.split(":")

    def brake_state_cb(self, msg: String):
        self.brake_error = msg.data.split(":")
        
    def robot_velocity_cb(self, msg: Float32):
        self.robot_velocity.data = msg.data
        
    def robot_steering_cb(self, msg: Float32):
        self.robot_steering.data = msg.data
        
    def emergency_brake_cb(self, msg:Bool):
        self.emergency_brake.data = msg.data
        

        
    def set_images(self):
        self.picture.setPixmap(QtGui.QPixmap(self.image_paths["main_picture"]))
        self.BR_speed_pic.setPixmap(QtGui.QPixmap(self.image_paths["speed_logo"]))
        self.FL_speed_pic.setPixmap(QtGui.QPixmap(self.image_paths["speed_logo"]))
        self.BL_speed_pic.setPixmap(QtGui.QPixmap(self.image_paths["speed_logo"]))
        self.BR_break_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_logo"]))
        self.BR_steering_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_logo"]))
        self.BL_break_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_logo"]))
        self.BL_steering_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_logo"]))
        self.FL_break_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_logo"]))
        self.FL_steering_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_logo"]))
        self.FR_break_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_logo"]))
        self.FR_steering_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_logo"]))
        self.FR_speed_pic.setPixmap(QtGui.QPixmap(self.image_paths["speed_logo"]))
        self.FR_break_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_error_logo"]))
        self.FR_steering_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_error_logo"]))
        self.FL_steering_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_error_logo"]))
        self.FL_break_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_error_logo"]))
        self.BR_steering_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_error_logo"]))
        self.BR_break_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_error_logo"]))
        self.BL_steering_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_error_logo"]))
        self.BL_break_error_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_error_logo"]))
        self.grey_color.setPixmap(QtGui.QPixmap(self.image_paths["grey_background"]))
        self.FL_FOV.setPixmap(QtGui.QPixmap(self.image_paths["FOV_up"]))
        self.FR_FOV.setPixmap(QtGui.QPixmap(self.image_paths["FOV_up"]))
        self.control_speed_pic.setPixmap(QtGui.QPixmap(self.image_paths["speed_logo"]))
        self.control_break_pic.setPixmap(QtGui.QPixmap(self.image_paths["break_logo"]))
        self.gear_pic.setPixmap(QtGui.QPixmap(self.image_paths["gear_logo"]))
        self.control_steering_pic.setPixmap(QtGui.QPixmap(self.image_paths["steering_logo"]))
        self.L_FOV.setPixmap(QtGui.QPixmap(self.image_paths["FOV_left"]))
        self.R_FOV.setPixmap(QtGui.QPixmap(self.image_paths["FOV_right"]))
        self.BR_FOV.setPixmap(QtGui.QPixmap(self.image_paths["FOV_down"]))
        self.BL_FOV.setPixmap(QtGui.QPixmap(self.image_paths["FOV_down"]))
        self.battery_pic.setPixmap(QtGui.QPixmap(self.image_paths["battery_logo"]))
    

    def update_labels(self):
        if self.motors_speed.data :
            self.FR_speed_label.setText(f"{self.motors_speed.data[0]} rpm")
            self.BR_speed_label.setText(f"{self.motors_speed.data[1]} rpm")
            self.FL_speed_label.setText(f"{self.motors_speed.data[2]} rpm")
            self.BL_speed_label.setText(f"{self.motors_speed.data[3]} rpm")

        if self.steering_angle.data:
            self.FR_steering_label.setText(f"{self.steering_angle.data[0]}°")
            self.FL_steering_label.setText(f"{self.steering_angle.data[1]}°")
            self.BR_steering_label.setText(f"{self.steering_angle.data[2]}°")
            self.BL_steering_label.setText(f"{self.steering_angle.data[3]}°")

        if self.brake_percentage.data:
            self.Fr_break_label.setText(f"{self.brake_percentage.data[0]}%")
            self.FL_break_label.setText(f"{self.brake_percentage.data[1]}%")
            self.BR_break_label.setText(f"{self.brake_percentage.data[2]}%")
            self.BL_break_label.setText(f"{self.brake_percentage.data[3]}%")
            
        if self.ultrasonic.data:
            self.FR_ultrasonic_label.setText(f"{self.ultrasonic.data[0]}")
            self.FL_ultrasonic_label.setText(f"{self.ultrasonic.data[1]}")
            self.BR_ultrasonic_label.setText(f"{self.ultrasonic.data[2]}")
            self.BL_ultrasonic_label.setText(f"{self.ultrasonic.data[3]}")
            self.R_ultrasonic_label.setText(f"{self.ultrasonic.data[4]}")
            self.L_ultrasonic_label.setText(f"{self.ultrasonic.data[5]}") 
         
        if self.steer_error:
            self.FR_steering_error_label.setText(self.steer_error[0])
            self.FL_steering_error_label.setText(self.steer_error[1])
            self.BR_steering_error_label.setText(self.steer_error[2])
            self.BL_steering_error_label.setText(self.steer_error[3])

        if self.brake_error:
            self.FR_break_error_label.setText(self.brake_error[0])
            self.FL_break_error_label.setText(self.brake_error[1])
            self.BR_break_error_label.setText(self.brake_error[2])
            self.BL_break_error_label.setText(self.brake_error[3])
            
        if self.robot_velocity.data:
            self.control_speed_label.setText(f"{self.robot_velocity.data} rpm")
            
        if self.robot_steering.data:
            self.control_steering_label.setText(f"{self.robot_steering.data}°")
            
        if self.emergency_brake.data:
            self.control_break_label.setText("True")
            self.control_break_label.setStyleSheet("color: red")
            
        if not self.emergency_brake.data:
            self.control_break_label.setText("False")
            self.control_break_label.setStyleSheet("color: white")
            
        if self.battery.voltage:
            self.batter_percentage_label.setText(f"{self.battery.percentage} %")
            self.battery_current_label.setText(f"{self.battery.current} A")
            self.battery_voltage_label.setText(f"{self.battery.voltage} V")
            
            

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    dialog = RobotVisualizer()
    dialog.show()
    sys.exit(app.exec())
