#!/usr/bin/env python3

import os
import subprocess
import rospy

import roslaunch
import rospkg
import rosnode
import rostopic

from std_msgs.msg import Float32, Int8MultiArray, Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from business_layer_pkg.srv import end_map, end_mapRequest, end_mapResponse

from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from business_layer_pkg.srv import (
    health_check,
    health_checkRequest,
    health_checkResponse,
)

from hdl_graph_slam.srv import SaveMap, SaveMapRequest


class RosArchitecture:
    def __init__(self, ros_dist):

        pass
    
    def node_init(self, node_name, anonymous=False):
        # rospy.init_node(node_name, anonymous=anonymous)
        pass
        
    def ros_publisher(self, topic_name, message_type, queue_size=1):
        # return rospy.Publisher(topic_name, message_type, queue_size=queue_size)
        pass

class ros1Architecture(RosArchitecture):
    def __init__(self, ros_dist):
        super().__init__(ros_dist)
        self.node_init("ros1_architecture")
        self.publisher = self.ros_publisher("/ros1_publisher", Float32, 1)
        self.publisher.publish(Float32(1.0))
        
        
if __name__ == "__main__":

    ros_dist = os.getenv("ROS_DISTRO")
    ros_architecture = RosArchitecture(ros_dist)
    rospy.init_node("ros_architecture")
    rospy.spin()
