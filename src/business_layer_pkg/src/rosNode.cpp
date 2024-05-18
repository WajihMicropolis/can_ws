#include "rosNode.hpp"

// bool add(business_layer_pkg::health_check::Request &req, business_layer_pkg::health_check::Response &res);

rosNode::rosNode(/* args */)
{
   this->_nh = new ros::NodeHandle();

   this->getParams();

   this->_robotStatePub = this->_nh->advertise<std_msgs::String>(this->_robotStateTopic, 1, true);
   this->_emergencyCausePub = this->_nh->advertise<std_msgs::String>(this->_emergencyCauseTopic, 1, true);
   this->_healthCheckPub = this->_nh->advertise<std_msgs::String>(this->_healthCheckTopic, 1, true);
   this->_connectionQualityPub = this->_nh->advertise<std_msgs::Int16>(this->_connectionQualityTopic, 1, true);
   this->_batteryPub = this->_nh->advertise<sensor_msgs::BatteryState>(this->_batteryTopic, 1, true);
   this->_gpsPub = this->_nh->advertise<sensor_msgs::NavSatFix>(this->_gpsTopic, 1, true);

   // Create the service and advertise it to the ROS computational network
   this->_healthCheckService = this->_nh->advertiseService(this->_healthCheckServiceName, &rosNode::healthCheck, this);
   this->_resetRobotStateService = this->_nh->advertiseService(this->_resetRobotStateServiceName, &rosNode::healthCheck, this);

   this->_requestMode = "RANDOM";

   this->_dummyData = new dummyData(this->_battery_discharge_time, this->_battery_charge_time, this->_robot_state_change_time, this->_robot_state_change_in_minutes, this->_health_check_time, this->_gps_change_time);
   this->_lastConnectionTime = ros::Time::now();
   ros::Rate loop_rate(2);
   for (size_t i = 0; i < 2; i++)
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}

void rosNode::getParams()
{
   ROS_INFO("[rosNode] Getting parameters from the parameter server");
   this->getRosParam("robot_state_topic", this->_robotStateTopic);
   this->getRosParam("robot_state_change_time", this->_robot_state_change_time);
   this->getRosParam("robot_state_change_in_minutes", this->_robot_state_change_in_minutes);
   this->getRosParam("reset_robot_state_service_name", this->_resetRobotStateServiceName);

   this->getRosParam("emergency_cause_topic", this->_emergencyCauseTopic);

   this->getRosParam("battery_topic", this->_batteryTopic);
   this->getRosParam("battery_discharge_time", this->_battery_discharge_time);
   this->getRosParam("battery_charge_time", this->_battery_charge_time);

   this->getRosParam("connection_quality_topic", this->_connectionQualityTopic);
   this->getRosParam("connection_quality_time", this->_connection_quality_time);

   this->getRosParam("healthCheckTopic", this->_healthCheckTopic);
   this->getRosParam("healthCheckServiceName", this->_healthCheckServiceName);
   this->getRosParam("health_check_time", this->_health_check_time);

   this->getRosParam("gps_topic", this->_gpsTopic);
   this->getRosParam("gps_change_time", this->_gps_change_time);
   ROS_INFO("[rosNode] Parameters are set");
}

void rosNode::getRosParam(std::string paramName, auto &paramValue)
{
   if (!this->_nh->getParam(paramName, paramValue))
   {
      ROS_WARN("[rosNode] [PARAM] %s is not set", paramName.c_str());
      exit(1);
   }
   std::stringstream strg;
   strg << paramValue;
   std::string s = strg.str();
   ROS_INFO("[rosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
}

bool rosNode::resetRobotState(std_srvs::Empty::Request &req,
                              std_srvs::Empty::Response &res)
{
   this->_requestMode = "RANDOM";
   this->_modesChecked = true, this->_resetRequest = false;
   ROS_WARN("RequestMode: %s", this->_requestMode.c_str());
   return true;
}

bool rosNode::healthCheck(business_layer_pkg::health_check::Request &req,
                          business_layer_pkg::health_check::Response &res)
{
   this->_resetRequest = true;
   this->_requestMode = req.nextMode;
   ROS_WARN("RequestMode: %s", this->_requestMode.c_str());
   this->_modeToBeChecked.clear();

   if (this->_requestMode == "STAND_BY")
   {
      for (size_t i = 0; i < (1); i++)
         this->_modeToBeChecked.push_back(this->_healthCheckArray[i]);
   }
   else if (this->_requestMode == "FREE_DRIVING")
   {
      for (size_t i = 0; i < (3); i++)
         this->_modeToBeChecked.push_back(this->_healthCheckArray[i]);
   }
   else if (this->_requestMode == "MAPPING")
   {
      for (size_t i = 0; i < (4); i++)
         this->_modeToBeChecked.push_back(this->_healthCheckArray[i]);
   }
   else if (this->_requestMode == "MISSION_OUTDOOR")
   {
      for (auto check : this->_healthCheckArray)
         this->_modeToBeChecked.push_back(check);
   }
   else if (this->_requestMode == "MISSION_INDOOR")
   {
      for (auto check : this->_healthCheckArray)
         this->_modeToBeChecked.push_back(check);
   }
   else
   {
      this->_requestMode = "RANDOM";
      this->_modesChecked = true;
      ROS_ERROR("RequestMode is EMPTY or UNDEFINED !!!");
      ROS_ERROR("Back to RANDOM MODE !!!");

      return false;
   }
   this->_modesChecked = false;
   res.checks = this->_modeToBeChecked;

   return true;
}

void rosNode::publishRobotState(std_msgs::String msg)
{
   if (msg.data == this->_prevRobotState)
      return;
   this->_prevRobotState = msg.data;
   ROS_INFO("Robot state: %s", msg.data.c_str());
   this->_robotStatePub.publish(msg);
}

void rosNode::publishEmergencyCause(std_msgs::String msg)
{
   if (msg.data == this->_prevEmergencyCause || msg.data == "")
      return;
   this->_prevEmergencyCause = msg.data;
   ROS_INFO("Emergency cause: %s", msg.data.c_str());
   this->_emergencyCausePub.publish(msg);
}

void rosNode::publishBatteryLevel(sensor_msgs::BatteryState batteryMsg)
{
   if (batteryMsg.percentage == this->_prevBatteryLevel)
      return;
   this->_prevBatteryLevel = batteryMsg.percentage;

   ROS_INFO("Battery percentage: %f %%", batteryMsg.percentage);
   ROS_INFO("Battery voltage: %f V", batteryMsg.voltage);
   this->_batteryPub.publish(batteryMsg);
}

// publish connectivity quality
void rosNode::publishConnectionQuality(std_msgs::Int16 quality)
{
   if (ros::Time::now().toSec() - this->_lastConnectionTime.toSec() < this->_connection_quality_time)
      return;

   ROS_INFO("-----------------");
   ROS_INFO("Connection quality: %d ms", quality.data);
   this->_connectionQualityPub.publish(quality);
   this->_lastConnectionTime = ros::Time::now();
}

void rosNode::publishGps(sensor_msgs::NavSatFix gpsMsg)
{
   if (gpsMsg.latitude == this->_prevGpsLat)
      return;
   this->_prevGpsLat = gpsMsg.latitude;
   this->_gpsPub.publish(gpsMsg);
}
void rosNode::publishHealthCheck(std_msgs::String msg)
{
   if ((msg.data == "" || msg.data == this->_prevHealthCheck))
      return;
   this->_prevHealthCheck = msg.data;

   this->_healthCheckPub.publish(msg);
}

void rosNode::publish()
{
   // make the robot state change every 5 seconds
   this->publishConnectionQuality(this->_dummyData->getConnectionQuality()); // publish connection quality
   this->publishBatteryLevel(this->_dummyData->getBattery());

   std_msgs::String robotState = this->_dummyData->getRobotState(this->_requestMode);
   this->publishRobotState(robotState);
   if (robotState.data == "EMERGENCY")
      this->publishEmergencyCause(this->_dummyData->getEmergencyCause());

   this->publishHealthCheck(this->_dummyData->getHealthCheck(this->_requestMode, this->_modeToBeChecked, this->_modesChecked, this->_resetRequest));
   this->publishGps(this->_dummyData->getGps());
}
rosNode::~rosNode()
{
   ros::shutdown();
}
