#ifndef __DUMMY_HPP__
#define __DUMMY_HPP__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;

class dummyData
{
private:
    /* data */
    #define maxLat 25.0331
    #define minLat 25.0324
    bool _goingForward;
    float _robotChangeStep;
    std::string _robotStates[9] = {"STAND_BY", "MISSION_OUTDOOR", "MISSION_INDOOR", "FREE_DRIVING", "PAUSED", "CHARGING", "UNDER_MAINTENANCE", "EMERGENCY", "MAPPING"};
    std::string _emergencyCause[3] = {"LOW_BATTERY", "STEERING_MOTOR_DAMAGE", "BRAKE_MOTOR_DAMAGE"};

    enum batteryState
    {
        charging,
        discharging
    } _batteryState;

    ros::Time _lastBatteryTime,
        _lastRobotStateTime,
        _lastHealthCheckTime,
        _lastGpsTime;

    int _batteryPercentage,
        _prevBatteryPercentage;

    float _battery_discharge_time,
        _battery_charge_time,
        _robot_state_change_time,
        _health_check_time,
        _gps_time;

    int _robotStateCounter,
        _emergencyCauseCounter;

    bool _robot_state_change_in_minutes,
        _emergencyFlag;

    sensor_msgs::BatteryState batteryMsg;
    sensor_msgs::NavSatFix _gpsMsg;
    string _type, _status, _failCause, _message;
    size_t i = 0, j = 0;

    std::string _robotState,
        _prevRobotState,
        _requestMode;
    std::string modeCheckStatus[4] = {"STARTED", "PENDING", "SUCCEEDED", "FAILED"};
    bool checked = false;

public:
    std_msgs::Int16 getConnectionQuality();
    sensor_msgs::BatteryState getBattery();
    std_msgs::String getRobotState(string requestMode);
    std_msgs::String getEmergencyCause();
    std_msgs::String getHealthCheck(string requestMode, std::vector<std::string> modeChecks, bool &modesChecked, bool &resetRequest);
    sensor_msgs::NavSatFix getGps();

    dummyData(float battery_discharge_time, float battery_charge_time, int robot_state_change_time, bool robot_state_change_in_minutes, float health_check_time, float gps_time);
    ~dummyData();
};

#endif // __DUMMY_HPP__