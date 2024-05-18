#include "integration.hpp"
#include "rosNode.hpp"
#include <geometry_msgs/Twist.h>

integration::integration(ros::NodeHandle & nh) 
{
   nh.subscribe("cmd_vel", 10, &integration::cmd_vel_cb, this);
//    this->_lastBatteryTime = this->_lastRobotStateTime = ros::Time::now();

}

void integration::cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    // std_msgs::String m;
    // m.data = this->_robotStates[2];

    // n.publishRobotState("MISSION_INDOOR");
    // this->publishRobotState(m);
    this->getRobotState();
}

bool integration::check_key_off()
{
    //some logic goes here 
    //return true if key is off
    //false if vechile is working
    return true;
}

bool integration::check_mission()
{
    //logic goes here
    //return true if mission is ongoing
    //return false if mission not ongoing

    //check move_base action to see if there is an ongoing mission or not
    return true;
}



std_msgs::String integration::getRobotState()
{
    //Logic goes here
    //Check no singals at all

    //the dummest solution is to make getRobotState as a sequencer
    //check every mode and return true when one return true

    //the best solution is to make a FSM or BT making use of the previous state
    //as you can only go to few states when you are stading at one state


    std_msgs::String m;

    if(this->check_key_off())
    {
        m.data = this->_robotStates[9];
    }

    //Check EMERGENCY it will either come form faulty sensor or 

    return m;
    
}
