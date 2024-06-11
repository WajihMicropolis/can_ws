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
    def __init__(self):

        pass
    
    def node_init(self, node_name, anonymous=False): pass
    def publisher(self, topic_name, message_type, queue_size=1, latch=False): pass
    def subscriber(self, topic_name, message_type, callback, queue_size=1): pass
    def service_server(self, service_name, service_type, handler): pass
    def service_client(self, service_name, service_type): pass
    def wait_for_service(self, service_name): pass
    def ros_topic_hz(self, window_size): pass
    def timer(self, period, callback): pass
    def time_now(self): pass
    def time_now_sec(self): pass
    def duration(self, duration): pass
    def spin(self): pass
    def is_shutdown(self): pass
    def sleep(self, duration): pass
    def get_param(self, param_name): pass


class ros1Architecture(RosArchitecture):
    def __init__(self):
        super().__init__()
        self.node_init("ros1_architecture")

    def node_init(self, node_name, anonymous=False):
        return rospy.init_node(node_name, anonymous=anonymous)
    
    def publisher(self, topic_name, message_type, queue_size=1, latch=False):
        return rospy.Publisher(topic_name, message_type, queue_size=queue_size, latch=latch)
    
    def subscriber(self, topic_name, message_type, callback, queue_size=1):
        return rospy.Subscriber(topic_name, message_type, callback, queue_size=queue_size)
    
    def service_server(self, service_name, service_type, handler):
        return rospy.Service(service_name, service_type, handler)
    
    def service_client(self, service_name, service_type):
        return rospy.ServiceProxy(service_name, service_type)
    
    def wait_for_service(self, service_name):
        return rospy.wait_for_service(service_name)
    
    def ros_topic_hz(self, window_size):
        return rostopic.ROSTopicHz(window_size=window_size)
    
    def timer(self, period, callback):
        return rospy.Timer(rospy.Duration(period), callback)
    
    def time_now(self):
        return rospy.Time.now()
    
    def time_now_sec(self):
        return rospy.Time.now().to_sec()
    
    def duration(self, duration):
        return rospy.Duration(duration)
    
    def spin(self):
        return rospy.spin()
    
    def is_shutdown(self):
        return rospy.is_shutdown()
    
    def sleep(self, duration):
        return rospy.sleep(duration)
    
    def get_param(self, param_name):
        return rospy.get_param(param_name)

    
    
    
        
        
if __name__ == "__main__":

    ros_dist = os.getenv("ROS_DISTRO")
    if ros_dist is None:
        print("ROS_DISTRO environment variable is not set")
        exit(1)
    if ros_dist == "noetic":
        ros = ros1Architecture()

    ros.node_init("ros1_architecture")
    pub = ros.publisher("/ros1_publisher", Float32, 1)
    pub.publish(Float32(1.0))
