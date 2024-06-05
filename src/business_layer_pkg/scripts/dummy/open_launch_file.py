#!/usr/bin/env python3

import re

from sympy import true
import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerResponse
import roslaunch
import time

def start_node_direct():
    """
    Does work as well from service/topic callbacks directly using rosrun
    """    
    package = 'octomap_server'
    node_name = 'octomap_mapping.launch'

    command = "roslaunch {0} {1} open_rviz:=false".format(package, node_name)

    p = subprocess.Popen(command, shell=True)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")


def start_node2():
    """
    Does work as well from service/topic callbacks using launch files
    """    
    package = 'YOUR_PACKAGE'
    launch_file = 'YOUR_LAUNCHFILE.launch'

    command = "roslaunch  {0} {1}".format(package, launch_file)

    p = subprocess.Popen(command, shell=True)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")

start = False
running = False
def start_node():
    global start, running
    if start == False:
        return
    else:
        start = False
        running = True
    print("start node")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/microspot/webots_dir/src/webots_ros/octomap_mapping/octomap_server/launch/octomap_mapping.launch"])
    launch.start()
    print(" check: ", launch.pm.is_shutdown)
    rospy.loginfo("started")

    # rospy.sleep(3)
    # 3 seconds later
    # launch.shutdown()




def service_callback(req):
    global start
    start = True
    # edit below for other options
    # start_node()
    #start_node2()
    # start_node_direct()

    return TriggerResponse(success=True)

def terminate_service_callback(req):
    command = "rosnode kill /octomap_server"
    
    p = subprocess.Popen(command, shell=True)

    state = p.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")
        
    return TriggerResponse(success=True)
if __name__ == '__main__':
    rospy.init_node('test1', anonymous=True)

    service = rospy.Service('launch', Trigger, service_callback)
    terminate = rospy.Service('terminate', Trigger, terminate_service_callback)

    while not rospy.is_shutdown():
        start_node()
        
    rospy.spin()