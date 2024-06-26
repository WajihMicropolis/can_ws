#include <nlohmann/json.hpp>
#include <ros/ros.h>
using json = nlohmann::json;

void robotStateCallback(const std_msgs::String::ConstPtr &msg)
{
   // cout<<"Robot state: "<<msg->data<<endl;
   // extract data from json
   std::string data = msg->data;
   // convert string to json
   json j = json::parse(data);
   int cq = j["connection_quality"];
   std::cout << "- Connection Quality: " << j["connection_quality"] << std::endl;
   json j_pub;
   j_pub["connection_quality"] = cq;
   std_msgs::String msg_pub;
   msg_pub.data = j_pub.dump();
   // publish the connection quality
   this->_robotStatePub.publish(msg_pub);

   cout<<"------------------"<<endl;
}