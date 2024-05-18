#include "dummy.hpp"

dummyData::dummyData(float battery_discharge_time, float battery_charge_time, int robot_state_change_time, bool robot_state_change_in_minutes, float health_check_time, float gps_time)
    : _battery_discharge_time(battery_discharge_time),
      _battery_charge_time(battery_charge_time),
      _robot_state_change_time(robot_state_change_time),
      _robot_state_change_in_minutes(robot_state_change_in_minutes),
      _health_check_time(health_check_time),
      _gps_time(gps_time)
{
   this->_robotStateCounter = 0;
   this->_batteryPercentage = 100;
   this->_emergencyFlag = false;
   this->_batteryState = discharging;
   this->_robot_state_change_time = this->_robot_state_change_in_minutes ? this->_robot_state_change_time * 60 : this->_robot_state_change_time;

   this->_lastBatteryTime = this->_lastRobotStateTime = ros::Time::now();
   this->_lastHealthCheckTime = ros::Time::now();
   this->_lastGpsTime = ros::Time::now();
   this->_goingForward = true;
   this->_gpsMsg.latitude = minLat;
   this->_robotChangeStep = (maxLat - minLat) / 100;
}

std_msgs::String dummyData::getHealthCheck(string requestMode, std::vector<std::string> modeChecks, bool &modesChecked, bool &resetRequest)
{
   std_msgs::String msg;
   if (this->_requestMode == "RANDOM" || modesChecked)
   {
      msg.data = "";
      return msg;
   }
   if (ros::Time::now().toSec() - this->_lastHealthCheckTime.toSec() < this->_health_check_time)
   {
      msg.data = "";
      return msg;
   }
   this->_lastHealthCheckTime = ros::Time::now();
   if (resetRequest)
   {
      i = 0;
      j = 0;
      resetRequest = false;
   }
   if (i < modeChecks.size())
   {
      _type = modeChecks[i];
      cout << "_type: " << _type << endl;
      if (j < 3)
      {
         _status = modeCheckStatus[j <= 1 ? j : i == (modeChecks.size() -1)? rand() % 2 + 2: j];
         cout << "_status: " << _status << endl;
         _message = _type + ":" + _status;
         if (_status == "FAILED")
            _message += ":error";
         j++;
      }
      else
      {
         i++;
         j = 0;
         cout << "--------------------" << endl;
      }
   }
   else
   {
      i = 0;
      j = 0;
      modesChecked = true;
   }

   msg.data = _message;
   return msg;
}

sensor_msgs::NavSatFix dummyData::getGps()
{
   if (ros::Time::now().toSec() - this->_lastGpsTime.toSec() < this->_gps_time)
   {
      return _gpsMsg;
   }
   this->_lastGpsTime = ros::Time::now();

   _gpsMsg.header.stamp = ros::Time::now();
   _gpsMsg.header.frame_id = "gps";
   _gpsMsg.longitude = 55.188369;
   _gpsMsg.altitude = 10.0;

   if (_goingForward && _gpsMsg.latitude < maxLat)
      _gpsMsg.latitude += this->_robotChangeStep;

   else if (_goingForward &&_gpsMsg.latitude >= maxLat)
      _goingForward = false;

   else if (!_goingForward && _gpsMsg.latitude > minLat)
      _gpsMsg.latitude -= this->_robotChangeStep;

   else if (!_goingForward &&_gpsMsg.latitude <= minLat)
      _goingForward = true;

   return _gpsMsg;
}

sensor_msgs::BatteryState dummyData::getBattery()
{
   this->_prevBatteryPercentage = this->_batteryPercentage;
   if (ros::Time::now().toSec() - this->_lastBatteryTime.toSec() >
       (_batteryState == discharging ? this->_battery_discharge_time : this->_battery_charge_time))
   {
      if (_batteryState == discharging)
      {
         this->_batteryPercentage > 0 ? this->_batteryPercentage-- : _batteryState = charging;
      }
      else
      {
         this->_batteryPercentage < 100 ? this->_batteryPercentage++ : _batteryState = discharging;
      }
      this->_lastBatteryTime = ros::Time::now();
   }
   bool charging = this->_batteryState == charging;

   batteryMsg.header.stamp = ros::Time::now();
   batteryMsg.header.frame_id = "battery";

   batteryMsg.percentage = this->_batteryPercentage;
   batteryMsg.voltage = (48 * this->_batteryPercentage) / 100.0;
   batteryMsg.current = rand() % 10 + 1;
   batteryMsg.temperature = 27.0;
   batteryMsg.capacity = 48;
   batteryMsg.design_capacity = 48;

   batteryMsg.charge = charging;
   batteryMsg.power_supply_status = charging;
   batteryMsg.power_supply_health = charging;
   batteryMsg.power_supply_technology = charging;
   return batteryMsg;
}

std_msgs::String dummyData::getEmergencyCause()
{
   std_msgs::String msg;
   // cout << "_batteryPercentage: " << _batteryPercentage << endl;
   // cout << "prevBatteryPercentage: " << _prevBatteryPercentage << endl;
   if (this->_emergencyFlag && this->_batteryPercentage < 30 && this->_batteryPercentage != this->_prevBatteryPercentage)
   {
      this->_emergencyCauseCounter = 0;
      ROS_INFO("Emergency cause: %s", this->_emergencyCause[this->_emergencyCauseCounter].c_str());

      msg.data = this->_emergencyCause[this->_emergencyCauseCounter];
      return msg;
   }
   else if (this->_emergencyFlag && this->_prevRobotState != "EMERGENCY")
   {
      this->_emergencyCauseCounter = this->_emergencyCauseCounter == 2 ? 1 : 2;
      ROS_INFO("Emergency cause: %s", this->_emergencyCause[this->_emergencyCauseCounter].c_str());

      msg.data = this->_emergencyCause[this->_emergencyCauseCounter];
      return msg;
   }
   return msg;
}

std_msgs::String dummyData::getRobotState(string requestMode)
{
   std_msgs::String msg;
   this->_requestMode = requestMode;

   this->_prevRobotState = this->_robotState;
   if (this->_batteryState == charging)
   {
      this->_robotState = this->_robotStates[5]; // robot is charging
      this->_emergencyFlag = false;
   }

   else if (this->_batteryPercentage < 30)
   {
      this->_robotState = this->_robotStates[7]; // robot is in emergency state low battery
      this->_emergencyFlag = true;
   }
   else
   {
      if (this->_requestMode != "RANDOM")
      {
         this->_robotState = this->_requestMode;
         msg.data = this->_robotState;
         return msg;
      } // robot is in requested state

      if (ros::Time::now().toSec() - this->_lastRobotStateTime.toSec() > this->_robot_state_change_time)
      {
         // if condition is true, increment the counter, if not, reset the counter to 0 and increment the counter by 2 to skip the 5th state
         this->_robotStateCounter = this->_robotStateCounter < 8 ? ++this->_robotStateCounter == 5 ? ++this->_robotStateCounter : this->_robotStateCounter : 0;
         this->_robotStateCounter == 7 ? this->_emergencyFlag = true : this->_emergencyFlag = false;
         this->_lastRobotStateTime = ros::Time::now();
      }
      this->_robotState = this->_robotStates[this->_robotStateCounter]; // robot is in mission outdoor
   }

   msg.data = this->_robotState;
   // ROS_INFO("Robot state: %s", msg.data.c_str());
   return msg;
}

std_msgs::Int16 dummyData::getConnectionQuality()
{
   // return random value between 75 and 100
   std_msgs::Int16 quality;
   quality.data = rand() % 25 + 75;
   return quality;
}

dummyData::~dummyData()
{
}
