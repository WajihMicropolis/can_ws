#ifndef __INTEGRATION_HPP__
#define __INTEGRATION_HPP__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
using namespace std;

class integration
{
private:
   /* data */
   std::string _robotStates[10] = {"STAND_BY", "MISSION_OUTDOOR", "MISSION_INDOOR", "FREE_DRIVING", "PAUSED", "CHARGING", "UNDER_MAINTENANCE", "EMERGENCY", "MAPPING", "KEY_OFF"};
   std::string _emergencyCause[3] = {"LOW_BATTERY", "STEERING_MOTOR_DAMAGE", "BRAKE_MOTOR_DAMAGE"};
   std::string _healthCheck[5] = {"CONNECTION_QUALITY", "HARDWARE", "BATTERY_LEVEL", "SENSOR", "AUTOPILOT"};

   enum batteryState
   {
      charging,
      discharging
   }_batteryState;

   ros::Time _lastBatteryTime,
       _lastRobotStateTime;

   int _batteryPercentage ,
       _battery_discharge_time,
       _battery_charge_time,
       _robot_state_change_time;

   int _robotStateCounter,
       _emergencyCauseCounter;

    bool _robot_state_change_in_minutes,
       _emergencyFlag;

   sensor_msgs::BatteryState batteryMsg;

   std::string _robotState,
       _prevRobotState;

public:
   std_msgs::Int16 getConnectionQuality();
   sensor_msgs::BatteryState getBattery();
   std_msgs::String getRobotState();
   std_msgs::String getEmergencyCause();
   
   bool check_key_off();
   bool check_mission();
   void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);

   integration(ros::NodeHandle & nh);
   ~integration();
};

#endif // __INTEGRATION_HPP__