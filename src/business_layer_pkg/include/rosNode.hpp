#ifndef __ROS_NODE_HPP__
#define __ROS_NODE_HPP__

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <business_layer_pkg/health_check.h>
#include "dummy.hpp"

using namespace std;

class rosNode
{
private:
    dummyData *_dummyData;

    ros::NodeHandle *_nh;
    ros::Publisher _robotStatePub,
        _emergencyCausePub,
        _batteryPub,
        _connectionQualityPub,
        _healthCheckPub,
        _gpsPub;


    ros::ServiceServer _healthCheckService,
        _resetRobotStateService;

    sensor_msgs::BatteryState _batteryState;

    std::string _robotStateTopic,
        _emergencyCauseTopic,
        _batteryTopic,
        _connectionQualityTopic,
        _healthCheckTopic,
        _healthCheckServiceName,
        _gpsTopic,
        _resetRobotStateServiceName;

    std::string _requestMode;
    std::vector<std::string> _modeToBeChecked;

    std::string _prevRobotState,
        _prevEmergencyCause,
        _prevHealthCheck;

    float _connection_quality_time,
        _battery_discharge_time,
        _battery_charge_time,
        _robot_state_change_time,
        _health_check_time,
        _gps_change_time,
        _prevGpsLat;

    float _prevBatteryLevel = 0;

    bool _robot_state_change_in_minutes;
    bool _modesChecked,
        _resetRequest,
        _newClient;

    ros::Time _lastConnectionTime;

    std::string _healthCheckArray[5] = {"CONNECTION_QUALITY", "HARDWARE", "BATTERY_LEVEL", "SENSOR", "AUTOPILOT"};

    bool healthCheck(business_layer_pkg::health_check::Request &req,
                     business_layer_pkg::health_check::Response &res);

    bool resetRobotState(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res);
    void getParams();
    void getRosParam(std::string paramName, auto &paramValue);

    void publishBatteryLevel(sensor_msgs::BatteryState batteryMsg);
    void publishConnectionQuality(std_msgs::Int16 quality);

    void publishRobotState(std_msgs::String msg);

    void publishEmergencyCause(std_msgs::String msg);
    void publishHealthCheck(std_msgs::String msg);

    void publishGps(sensor_msgs::NavSatFix gpsMsg);
    void chechForNewSubscriber();

public:
    void publish();

    rosNode(/* args */);
    ~rosNode();
};

#endif // __ROS_NODE_HPP__