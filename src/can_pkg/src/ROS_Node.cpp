#include "ROS_Node.hpp"

ROS_Node::ROS_Node(/* args */)
{
   this->_nh = new ros::NodeHandle(); // Create a Node Handle for the ROS

   this->_velocity_sub = this->_nh->subscribe<std_msgs::Float32>("robot/velocity", 1, [&](const std_msgs::Float32::ConstPtr &velocityMsg)
                                                                 {
                                                                    this->_velocity = velocityMsg->data;
                                                                  //   std::cout<<"velocity: " <<this->_velocity<<std::endl; 
                                                                    });

   this->_steering_sub = this->_nh->subscribe<std_msgs::Float32>("robot/steering", 1, [&](const std_msgs::Float32::ConstPtr &steeringMsg)
                                                                 {
                                                                    this->_steering = steeringMsg->data;
                                                                     // std::cout<<"steering: " <<this->_steering<<std::endl; 
                                                                     });

   this->_emergency_brake_sub = this->_nh->subscribe<std_msgs::Bool>("robot/emergency_brake", 1, [&](const std_msgs::Bool::ConstPtr &emergencyBrakeMsg)
                                                                     {
                                                                        _emergency_brake = emergencyBrakeMsg->data;
                                                                        // std::cout<<"emergency_brake: " <<_emergency_brake<<std::endl;
                                                                        if (emergencyBrakeMsg->data)
                                                                           this->_velocity = 0, this->_steering = 0; });

   this->_door_control_sub = this->_nh->subscribe<std_msgs::Bool>("robot/door_control", 1, [&](const std_msgs::Bool::ConstPtr &doorControlMsg)
                                                                  { _door_control = doorControlMsg->data;
                                                                  //   std::cout<<"door_control: " <<_door_control<<std::endl;
                                                                     });

   this->_motorsSpeed_pub        = this->_nh->advertise<std_msgs::Int16MultiArray>("motors_speed", 1,1);
   this->_steering_angle_pub     = this->_nh->advertise<std_msgs::Int16MultiArray>("steering_angle", 1,1);
   this->_brake_percentage_pub   = this->_nh->advertise<std_msgs::Int16MultiArray>("brake_percentage", 1,1);
   this->_ultrasonic_pub         = this->_nh->advertise<std_msgs::Int16MultiArray>("ultrasonic", 1,1);
   this->_rpy_pub                = this->_nh->advertise<std_msgs::Float32MultiArray>("rpy", 1,1);
   
   this->_battery_pub            = this->_nh->advertise<sensor_msgs::BatteryState>("battery", 1,1);
   this->_robot_speed_pub        = this->_nh->advertise<std_msgs::Float32>("robot_speed", 1,1);
   this->_door_state_pub         = this->_nh->advertise<std_msgs::String>("door_state", 1,1);
   this->_steering_error_pub     = this->_nh->advertise<std_msgs::String>("steering_status", 1,1);
   this->_brake_error_pub        = this->_nh->advertise<std_msgs::String>("brake_status", 1,1);
   this->_imu_pub                = this->_nh->advertise<sensor_msgs::Imu>("imu", 1,1);
   
   this->_steering_health_check_pub = this->_nh->advertise<std_msgs::Bool>("steering_health_check", 1,1);
   this->_braking_health_check_pub  = this->_nh->advertise<std_msgs::Bool>("braking_health_check", 1,1);

   this->_can_feedback.motors_speed.data.resize(4);
   this->_can_feedback.steering_angle.data.resize(4);
   this->_can_feedback.brake_percentage.data.resize(4);
   this->_can_feedback.ultrasonic.data.resize(6);

   this->getRosParam("/can_node/port", this->_USB_PORT);
   this->getRosParam("/can_node/publish_rate", this->publish_rate);
   this->getRosParam("/can_node/send_rate", this->send_rate);
   can_interface = new CAN_Interface((char *)this->_USB_PORT.c_str());
   ROS_INFO("[RosNode] ready to start");
}


void ROS_Node::update()
{
   can_interface->getFeedback(_can_feedback);

   if (ros::Time::now().toSec() - this->_publishTime.toSec() > 1.0 / publish_rate)
   {
      this->publishFeedback(_can_feedback);
      this->_publishTime = ros::Time::now();
   }

   if (ros::Time::now().toSec() - this->_sendTime.toSec() > 1.0 / send_rate)
   {
      can_interface->sendCmdVel(this->_velocity, this->_steering);
      can_interface->sendEmergencyBrake(this->_emergency_brake);
      can_interface->sendDoorControl(this->_door_control);
      this->_sendTime = ros::Time::now();
   }
   // this->printOnTerminal();
}

void ROS_Node::publishFeedback(CAN_Interface::CANFeedback &feedback)
{
   _motors_speed_msg = feedback.motors_speed;
   _steering_angle_msg = feedback.steering_angle;
   _brake_percentage_msg = feedback.brake_percentage;
   _ultrasonic_msg = feedback.ultrasonic;

   _rpy_msg = feedback.rpy;

   _imu_msg = feedback.imu;
   _imu_msg.header.stamp = ros::Time::now();
   _imu_msg.header.frame_id = "imu";

   _battery_state_msg = feedback.battery_state;
   _battery_state_msg.header.stamp = ros::Time::now();
   _battery_state_msg.header.frame_id = "battery";

   _robot_speed_msg.data = feedback.robot_speed;
   _door_state_msg = feedback.door_state;
   _steering_error_msg = feedback.steering_status;
   _brake_error_msg = feedback.braking_status;
   _steering_health_check_msg = feedback.steering_health_check;
   _braking_health_check_msg = feedback.braking_health_check;


   _imu_pub.publish(_imu_msg);
   _rpy_pub.publish(_rpy_msg);
   _motorsSpeed_pub.publish(_motors_speed_msg);
   _steering_angle_pub.publish(_steering_angle_msg);
   _steering_error_pub.publish(_steering_error_msg);
   _steering_health_check_pub.publish(_steering_health_check_msg);
   _brake_percentage_pub.publish(_brake_percentage_msg);
   _brake_error_pub.publish(_brake_error_msg);
   _braking_health_check_pub.publish(_braking_health_check_msg);
   _ultrasonic_pub.publish(_ultrasonic_msg);
   _battery_pub.publish(_battery_state_msg);
   _robot_speed_pub.publish(_robot_speed_msg);
   _door_state_pub.publish(_door_state_msg);

}

void ROS_Node::getRosParam(std::string paramName, auto &paramValue)
{
   if (!this->_nh->getParam(paramName, paramValue))
   {
      ROS_WARN("[RosNode] [PARAM] %s is not set", paramName.c_str());
      exit(1);
   }
   std::stringstream strg;
   strg << paramValue;
   std::string s = strg.str();
   ROS_INFO("[RosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
}
void ROS_Node::printOnTerminal()
{
   system("clear");
   std::cout << "---------------------- " << std::endl;
   std::cout << "\t velocity: " << this->_velocity << std::endl;
   std::cout << "\t steering: " << this->_steering << std::endl;
}

ROS_Node::~ROS_Node()
{
   this->_nh->shutdown();
}
