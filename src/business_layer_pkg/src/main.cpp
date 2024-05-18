#include "rosNode.hpp"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "Dummy_data_node", ros::init_options::AnonymousName);

   rosNode node;
   ros::Rate loop_rate(30);
   while (ros::ok())
   {
      node.publish();
      ros::spinOnce();
      loop_rate.sleep();
   }

   ros::shutdown();
   return 0;
}
