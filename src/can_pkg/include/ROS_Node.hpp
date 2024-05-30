#ifndef __ROS_NODE_HPP__
#define __ROS_NODE_HPP__

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Range.h"
#include "business_layer_pkg/health_check.h"
#include "CAN_Interface.hpp"

class ROS_Node
{
private:
    ros::NodeHandle *_nh;

    ros::Subscriber _velocity_sub,
        _steering_sub,
        _emergency_brake_sub,
        _door_control_sub,
        _robot_state_sub;

    ros::Publisher _motorsSpeed_pub,
        _steering_angle_pub,
        _brake_percentage_pub,
        _ultrasonic_pub,
        _battery_pub,
        _robot_speed_pub,
        _imu_pub,
        _rpy_pub,
        _door_state_pub,
        _lifter_state_pub,
        _drone_base_state_pub,
        _steering_state_pub,
        _brake_state_pub,
        _steering_health_check_pub,
        _braking_health_check_pub,
        _drive_mode_pub,
        _front_right_ultrasonic_pub,
        _front_left_ultrasonic_pub,
        _back_right_ultrasonic_pub,
        _back_left_ultrasonic_pub;

    ros::ServiceClient _health_check_client;
    business_layer_pkg::health_check _health_check_srv;
    std::string _robot_state;

    ros::Time _sendTime,
        _publishTime;

    Int16MultiArray _motors_speed_msg,
        _steering_angle_msg,
        _brake_percentage_msg,
        _ultrasonic_msg;

    Float32MultiArray _rpy_msg;

    Imu _imu_msg;
    BatteryState _battery_state_msg;
    Range _front_right_ultrasonic_msg,
        _front_left_ultrasonic_msg,
        _back_right_ultrasonic_msg,
        _back_left_ultrasonic_msg;

    Float32 _robot_speed_msg;

    CAN_Interface *can_interface;
    CAN_Interface::CANFeedback _can_feedback;

    std::string _USB_PORT;

    std_msgs::String _steering_state_msg,
        _brake_state_msg,
        _door_state_msg,
        _lifter_state_msg,
        _drone_base_state_msg,
        _drive_mode_msg;

    std_msgs::Bool _steering_health_check_msg,
        _braking_health_check_msg;

    int publish_rate,
        send_rate;
    float _velocity = 0.0,
          _steering = 0.0;
    float _prev_velocity = _velocity,
          _prev_steering = _steering;

    int idle_counter = 0;
    float _speed_fb = 0.0;
    bool _emergency_brake = false;
    uint8_t _door_control = false;

    void getRosParam(std::string paramName, auto &paramValue);
    void printOnTerminal();
    void publishFeedback(CAN_Interface::CANFeedback &feedback);
    void robotStateCallback(const std_msgs::String::ConstPtr &robotStateMsg);

public:
    ROS_Node(/* args */);
    ~ROS_Node();
    void update();
};

#endif //__ROS_NODE_HPP__