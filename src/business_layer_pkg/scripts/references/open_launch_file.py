#!/usr/bin/env python3

import re

from sympy import true
import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerResponse
from hdl_graph_slam.srv import SaveMap,SaveMapRequest ,SaveMapResponse
from sensor_msgs.msg import PointCloud2 
import roslaunch
import rosnode
import rostopic

import rospkg
import time

map_state = False

def start_node_direct():
    global map_state
    map_state = False
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_file_path = roslaunch.rlutil.resolve_launch_arguments(['hdl_graph_slam', 'hdl_graph_slam.launch'])[0]
    launch_files = [launch_file_path]

    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()

def end_node_direct():
    
    
    print("save map")
    rospy.wait_for_service('/hdl_graph_slam/save_map')
    print("service ready")
    map_req = SaveMapRequest()
    map_req.utm = False
    map_req.resolution = 0.1
    map_name = "map"
    map_req.destination = f"/home/microspot/can_ws/{map_name}.pcd"
    print("map_dest: ", map_req.destination)
    
    map_result = _save_map_srv(map_req)
    print("map_result: ", map_result)

        
    # kill_node = rosnode.kill_nodes(['/hdl_nodelet_manager'])
    
    # print("kill node: ", kill_node)
    # if kill_node[0] and not kill_node[1]:
    #     rospy.loginfo("Node killed successfully")
    # else:
    #     rospy.logerr("Node not killed successfully")
        
def node_ping(node_name = "/hdl_nodelet_manager", count=1, timeout=3):
    n = rosnode.rosnode_ping("/hdl_nodelet_manager", 1)
    print("node ping: ",n)
    
    
        
def service_callback(req):
    global map_state
    map_state = True
    # start_node_direct()
    return TriggerResponse(success=True)

def terminate_service_callback(req):
    end_node_direct()
    return TriggerResponse(success=True)
   
rt = rostopic.ROSTopicHz(-1)     
def timer_callback(event):
    global rt
    hz_data = rt.get_hz()
    if hz_data:
        rate = hz_data[0]
        rospy.loginfo("Publish rate: {:.2f} Hz".format(rate))
    else:
        rospy.loginfo("No messages received.")
        
if __name__ == '__main__':
    rospy.init_node('launch_node', anonymous=True)
    # Subscribe to the topic and set the callback to rt.callback_hz
    sub = rospy.Subscriber("/cloud", PointCloud2, rt.callback_hz)
    timer = rospy.Timer(rospy.Duration(1), timer_callback)  # Check the rate every second
    # Give some time for the subscriber to receive messages and calculate the rate
    _save_map_srv       = rospy.ServiceProxy("/hdl_graph_slam/save_map", SaveMap)
    
    
    
    service = rospy.Service('launch', Trigger, service_callback)
    terminate = rospy.Service('terminate', Trigger, terminate_service_callback)
    while not rospy.is_shutdown():

        if map_state:
            start_node_direct()
        node_ping()
        rospy.sleep(0.2)
        pass
    
    
    """            command = f"bash -c ' rosrun map_server map_saver -f /home/microspot/catkin_ws/maps/{req.map_name} map:=/projected_map'"
            result = subprocess.run(command, capture_output=True, text=True, shell=True,timeout=3)
            if 'Done' in result.stdout:
                print("Map saved successfully.")
            else:
                print("Map saving failed. Check the output for details.")
                print("stdout:", result.stdout)
                print("stderr:", result.stderr)
                return end_mapResponse(success=False, status_message="map not saved")
    """