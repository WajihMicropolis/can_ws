#!/usr/bin/env python3

import json
from copy import deepcopy
from os import error
from re import S

from sympy import false, true
import rospy

import roslaunch
import rospkg
import rosnode
import rostopic

from std_msgs.msg import Float32, Int8MultiArray, Bool, String, Int32
from business_layer_pkg.msg import StringArray
from business_layer_pkg.srv import Trigger, TriggerResponse
from business_layer_pkg.srv import end_map, end_mapRequest, end_mapResponse
from business_layer_pkg.msg import data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

from business_layer_pkg.srv import new_health_check, new_health_checkRequest, new_health_checkResponse

from hdl_graph_slam.srv import SaveMap, SaveMapRequest

from robot_control import Robot_Control
from robot_feedback import RobotFeedback
import requests

_healthCheckArray = [
    "CONNECTION_QUALITY",
    "HARDWARE",
    "BATTERY_LEVEL",
    "SENSOR",
    "AUTOPILOT",
]
_nextModeArray = [
    "STAND_BY",
    "FREE_DRIVING",
    "MAPPING",
    "MISSION_INDOOR",
    "MISSION_OUTDOOR",
]

_modeCheckStatus = ["STARTED", "PENDING", "SUCCEEDED", "FAILED"]


class Robot_Node:
    def __init__(self):
        rospy.init_node("robot_node", anonymous=True)
        self.ip = self.get_public_ip()
        self.pcl_topic = self.get_param("/robot_control/pcl_topic")
        self.autonomous_cmd_topic = self.get_param("/robot_control/autonomous_cmd_topic")
        
        #! publishers to the Teleoperation software
        self._robot_state_pub               = rospy.Publisher("robot_state", String, queue_size=1, latch=True)
        self._health_check_pub              = rospy.Publisher("health_check", String, queue_size=1, latch=True)
        # self._emergency_cause_pub           = rospy.Publisher("emergency_cause", String, queue_size=1, latch=True)
        self._emergency_cause_array_pub     = rospy.Publisher("emergency_cause", StringArray, queue_size=1, latch=True)
        self._robot_operational_details_pub = rospy.Publisher("robot_operational_details", String, queue_size=1, latch=True)
        #! subscribers from the Teleoperation software
        self._gear_sub           = rospy.Subscriber("gear", String, self.gear_cb)
        self._wasdb_sub          = rospy.Subscriber("wasdb", String, self.wasdb_cb)
        self._door_control_sub   = rospy.Subscriber("door_control", String, self.door_control_cb)
        self._ros_bridge_cli     = rospy.Subscriber("/client_count", Int32, lambda msg: setattr(self, 'connected_clients', msg.data))
        
        #! publishers to the Robot
        self.tele_operator_pub      = rospy.Publisher("teleop_cmd_vel", Twist, queue_size=1, latch=True)
        self.velocity_pub           = rospy.Publisher("robot/velocity",Float32,queue_size=1 ,latch=True)
        self.steering_pub           = rospy.Publisher("robot/steering",Float32,queue_size=1,latch=True)
        self.emergency_brake_pub    = rospy.Publisher("robot/emergency_brake",Bool,queue_size=1,latch=True)
        self.robot_door_control_pub = rospy.Publisher("robot/door_control",Int8MultiArray,queue_size=1,latch=True)
        #! subscribers from auto-pilot
        self._auto_pilot_sub    = rospy.Subscriber(self.autonomous_cmd_topic, Twist, self.autonomous_cmd_vel_cb)
        self.rt                 = rostopic.ROSTopicHz(-1)
        self._pcl_sub           = rospy.Subscriber(self.pcl_topic, PointCloud2, self.rt.callback_hz)
        self.timer              = rospy.Timer(rospy.Duration(1), self.timer_callback)  # Check the rate every second 

        #! services
        self._health_check_srv  = rospy.Service("health_check_srv", new_health_check, self.health_check_srv)
        self._streams_srv       = rospy.Service("streams_ids", Trigger, self.req_streames_srv)
        self._start_map_srv     = rospy.Service("start_map_srv", Trigger, self.start_map_srv)
        self._end_map_srv       = rospy.Service("end_map_srv", end_map, self.end_map_srv)
        self.retry_save_map     = rospy.Service("retry_save_map_srv", Trigger, self.retry_save_map_srv)
        self._save_map_srv      = rospy.ServiceProxy("/hdl_graph_slam/save_map", SaveMap)

        #! variables
        self.teleoperator_command   = Twist()
        self.auto_pilot_command     = Twist()
        self.robot_command          = Twist()
        
        self.robot_velocity_rpm             = Float32()
        self.robot_steering                 = Float32()
        self.robot_emergency_brake          = Bool()
        self.prev_robot_emergency_brake     = Bool()
        self.robot_operational_details      = String()
        self.prev_robot_operational_details = String()
        self.door_control                   = Int8MultiArray()
        self.prev_door_control              = Int8MultiArray()
        
        self.prev_robot_emergency_brake.data = False
        self.websocket_connection = True

        self.robot_state = "STAND_BY"
        self.next_mode   = deepcopy(self.robot_state)
        self.prev_robot_state = ""
        self.old_robot_state = ""

        self.connection_quality     = 75
        self.steering_health_check  = True
        self.braking_health_check   = True
        self.battery_capacity       = 100
        self.health_check_success   = 0
        self.pcl_rate               = 0
        self.connected_clients      = 0

        self._modeToBeChecked = []
        self.gear = 1
        
        self.door_control.data = [2,0,0,0,0]
        self.prev_door_control.data = deepcopy(self.door_control.data)
        
        self.Robot_Control  = Robot_Control()
        self.Robot_Feedback = RobotFeedback(self.ip)

        self.drive_data = {"w": 0, "a": 0, "s": 0, "d": 0, "b": 0}

        self.check_index, self.status_index = 0 , 0
        self.check_time = 0.3
        self.prev_health_check_time = rospy.Time.now()

        self.emergency_cause_array = []
        self.prev_emergency_cause_array = None
        
        self.emergency_check_time = 0.5
        self.prev_emergency_cause_time = rospy.Time.now()


        self.prev_publish_time = rospy.Time.now()
        self.elapsed_init_time = rospy.Time.now().to_sec()
        
        self.start_mapping_srv = False
        self.mapping = False
        self.i= 0
        self.pod_doors_control = {
            "top":2,
            "front_right": 0,
            "front_left": 0,
            "back_right": 0,
            "back_left": 0
        }
        self.door_pulse_timer = rospy.Time.now()
        self.publish_pulse = False
        print("Robot Node is ready, ", rospy.Time.now().to_sec())

    def get_public_ip(self):
        try:
            response = requests.get('https://api.ipify.org?format=json')
            response.raise_for_status()  # Raise an exception for HTTP errors
            ip_info = response.json()
            return ip_info['ip']
        except requests.RequestException as e:
            print(f"Error occurred: {e}")
            return None

    def map_value(self,x, a, b, c, d):
        """
        Maps a value x from range [a, b] to range [c, d].

        Parameters:
        - x: The value to map.
        - a: The lower bound of the original range.
        - b: The upper bound of the original range.
        - c: The lower bound of the new range.
        - d: The upper bound of the new range.

        Returns:
        - The value of x mapped to the new range [c, d].
        """
        return float(c + (x - a) * (d - c) / (b - a))
       
    def timer_callback(self, event):
        hz_data = self.rt.get_hz()
        if hz_data:
            self.pcl_rate = hz_data[0]
        else:
            self.pcl_rate = 0
       
    def retry_save_map_srv(self,req):
        pass
    
    def get_param(self, param_name):
        if not rospy.has_param(param_name):
            rospy.logerr(f"Parameter '{param_name}' not found")
            return
        param_value = rospy.get_param(param_name)     
        print(f"{param_name}: {param_value}")
        return param_value
        
    #* Launch/Run/Kill ROS package  
    def launch_pkg(self, pkg_name, launch_file):
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch_file_path = roslaunch.rlutil.resolve_launch_arguments([pkg_name,launch_file])[0]
        launch_files = [launch_file_path]

        launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        launch.start()
        return launch
    
    def run_node(self, pkg_name, executable, args = ''):

        node = roslaunch.core.Node(pkg_name, executable,args=args )

        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        process = launch.launch(node)
        print("process: ",process)
    
    def kill_node(self, node_name):
        kill_node = rosnode.kill_nodes([node_name])
        print("kill node: ", kill_node)
        if kill_node[0] and not kill_node[1]:
            rospy.loginfo(f"{node_name} Node killed successfully")
            return True
        
        return False
    
    #* Service Callbacks 
    def health_check_srv(self, req: new_health_checkRequest):
        self.next_mode = req.nextMode
        
        #! or self.next_mode == self.robot_state
        if self.next_mode not in _nextModeArray :
            rospy.logerr(f"Invalid : {self.next_mode}")
            return new_health_checkResponse(checks=[] , success=False, message="Invalid mode")

        rospy.logdebug(f"health_check_srv: {self.next_mode}")
        self._modeToBeChecked.clear()

        # self.robot_state = self.next_mode
        if self.next_mode == "STAND_BY":
            self._modeToBeChecked.append(_healthCheckArray[0])

        elif self.next_mode == "FREE_DRIVING":
            for i in range(3):
                self._modeToBeChecked.append(_healthCheckArray[i])

        elif self.next_mode == "MAPPING":
            self._modeToBeChecked.append(_healthCheckArray[3])
            # todo : add check for server connection to save map
        elif self.next_mode == "MISSION_INDOOR":
            self._modeToBeChecked.append(_healthCheckArray[4])

        elif self.next_mode == "MISSION_OUTDOOR":
            self._modeToBeChecked.append(_healthCheckArray[4])

        print("mode to be checked: ", self._modeToBeChecked)
        self.status_checks = {
            "CONNECTION_QUALITY": lambda: self.connection_quality >= 70,
            "HARDWARE": lambda: self.steering_health_check
            and self.braking_health_check,
            "BATTERY_LEVEL": lambda: self.battery_capacity > 30,
            "SENSOR": lambda: self.pcl_rate > 0,
        }
        self.error_messages = {
            "CONNECTION_QUALITY": "connection error",
            "HARDWARE": "steering error" if not self.steering_health_check else "brake error" if not self.braking_health_check else "hardware error",
            "BATTERY_LEVEL": "low battery",
            "SENSOR": "lidar not working",
        }

        checks = self._modeToBeChecked
        # Create a dictionary with the array nested under the "checks" key
        checks_dict = {"checks": checks}
        checks_json = json.dumps(checks_dict)

        # Print the JSON string
        print(checks_json)
        
        response = new_health_checkResponse()
        response.data = checks_json
        response.success = True
        return response

    def req_streames_srv(self,req):
        public_ip = self.ip
        print("public ip: ", public_ip)
        streams_ids = {
            "streams": [
                { 
                    "name": "Front Camera",
                    "type": "front", 
                    "connection_id": {
                        "address": public_ip,
                        "port": 8555,
                        "stream_name": "front_camera"
                    }
                },
                {
                    "name": "Back Camera", 
                    "type": "back", 
                    "connection_id": {
                        "address": public_ip,
                        "port": 8556,
                        "stream_name": "back_camera"
                    }
                }
            ]
        }

        return TriggerResponse(success=True, data=json.dumps(streams_ids), message="streams ids")

    def start_map_srv(self,req):      
        # self.start_mapping_srv = True
        print("Start MAPPING")
        return TriggerResponse(success=True, message="map started")

    def launch_map_server(self):
        if not self.start_mapping_srv:
            return
        hdl_pkg_name = "hdl_graph_slam"
        hdl_launch_file = "hdl_graph_slam.launch"
        self.launch_pkg(hdl_pkg_name, hdl_launch_file)
        
        pcd_pkg_name = "point_cloud_to_occupancy_grid"
        pcd_launch_file = "pcd_to_occupancy_grid.launch"
        self.launch_pkg(pcd_pkg_name, pcd_launch_file)
        
        self.start_mapping_srv = False
        self.mapping = True
        
    def end_map_srv(self,req: end_mapRequest):
        # if not self.mapping:
        #     return end_mapResponse(success=False, message="map not started")
        
        print("----------save map---------")
        print("req save map: ",req.save_map)
        print("req map name: ",req.map_name)
        self.map_name = req.map_name
        
        if req.save_map:
            if not req.map_name:
                return end_mapResponse(success=False, message="map name is required")
            # print("save map")
            # rospy.wait_for_service('/hdl_graph_slam/save_map')
            # print("service ready")
            # save_maps_destination = "/home/microspot/can_ws"
            # map_req = SaveMapRequest()
            # map_req.utm = False
            # map_req.resolution = 0.1
            # map_req.destination = f"{save_maps_destination}/{req.map_name}.pcd"
            # map_result = self._save_map_srv(map_req)
            # print("map_result: ", map_result)
            
            # self.run_node("map_server","map_saver",f"-f {save_maps_destination}/map")
            # todo save map server
            
            
        
        # pcd_to_grid_kill_node = self.kill_node("/point_cloud_to_occupancy_grid")
        # hdl_kill_node = self.kill_node("/hdl_nodelet_manager")
        
        # if not pcd_to_grid_kill_node:
        #     rospy.logerr("point_cloud_to_occupancy_grid not killed")
        #     return end_mapResponse(success=False, message="pcd_to_grid_kill_node not ended")
        
        # if not hdl_kill_node:
        #     rospy.logerr("hdl_nodelet_manager not killed")
        #     return end_mapResponse(success=False, message="hdl_kill_node not ended")
        
        rospy.loginfo("Nodes killed successfully")
        self.mapping = False
        self.robot_state = "FREE_DRIVING"
        file_name = {
            "files":{
                "image_path":"/home/microspot/can_ws/map.pgm",
                "yaml_path":"/home/microspot/can_ws/map.yaml",
                "pcd_path":f"/home/microspot/{req.map_name}.pcd"
                }
            }
        return end_mapResponse(success=True, message="mapping node ended", data = json.dumps(file_name))
        

    def check_node(self,node_name):
        node_ping = rosnode.rosnode_ping(node_name, 1, verbose = False)
        # print("node ping: ",n)
        return node_ping
    

    def autonomous_cmd_vel_cb(self, msg: Twist):
        self.auto_pilot_command.linear = msg.linear
        # map the steering angle from rad to degree
        self.auto_pilot_command.angular.z = self.map_value(msg.angular.z, -0.25, 0.25, -16, 16)
        self.auto_pilot_command.angular.z = min(16,max(-16,self.auto_pilot_command.angular.z))
        
    def door_control_cb(self, msg: String):
        print("door_control_cb: ", msg.data)
        if not msg.data:
            return
        
        if not hasattr(self, 'pod_doors_control'):
            rospy.logerr("Variable 'pod_doors_control' is not defined")
            return

        _door_control = json.loads(msg.data)
        _target_door = _door_control["target_door"]
        _target_state = _door_control["target_state"]
        
        print("target_door: ", _target_door)
        print("target_state: ", _target_state)
        
        if _target_door == "top":        
            self.pod_doors_control[_target_door] = 0 if _target_state =="open" else 2
        else:
            self.pod_doors_control[_target_door] = 1 if _target_state =="open" else 0
        # print(self.pod_doors_control)
        self.door_control.data = [self.pod_doors_control["top"] ,self.pod_doors_control["front_right"], self.pod_doors_control["front_left"], self.pod_doors_control["back_right"], self.pod_doors_control["back_left"]]
        

    def gear_cb(self, msg: String):

        gear = json.loads(msg.data)
        self.gear = gear["gear"]
        self.Robot_Control.gear_update(self.gear)

    def wasdb_cb(self, msg: String):

        wasdb = json.loads(msg.data)
        wasdb = wasdb["wasdb"]
        self.drive_data = wasdb
        self.Robot_Control.wasdb_update(self.drive_data)
    
    
    def publish_health_check(self):

        self.steering_health_check, self.braking_health_check = self.Robot_Feedback.getHealthCheck()
        
        if self._modeToBeChecked == []:
            return

        if rospy.Time.now() - self.prev_health_check_time < rospy.Duration(self.check_time):
            return
        self.prev_health_check_time = rospy.Time.now()
        

        if self.check_index == len(self._modeToBeChecked): #? When all the checks are done
            self.robot_state = self.next_mode if self.health_check_success == (len(self._modeToBeChecked)*3) else self.robot_state
            self.check_index, self.status_index, self.health_check_success = 0, 0, 0
            
            self._modeToBeChecked = []
            return
        
        check = self._modeToBeChecked[self.check_index] #? CONNECTION_QUALITY, HARDWARE, BATTERY_LEVEL, SENSOR
        
        if self.status_index == (len(_modeCheckStatus) -1): #? START, PENDING, SUCCEEDED or FAILED
            self.status_index = 0
            self.check_index += 1
            return
            
        status = _modeCheckStatus[self.status_index]
        self.health_check_success += 1
        
        if self.status_index == 2 and not self.status_checks[check]: #? in case of error in the check change the status to FAILED 
            status = _modeCheckStatus[3]
            status += f":{self.error_messages[check]}"
            self.health_check_success -=1
            
        self._health_check_pub.publish(f"{check}:{status}")
        self.status_index += 1

                
    def publish_doors_control(self):
        
        if self.door_control.data != self.prev_door_control.data:
            print("door_control: ", self.door_control.data)
            print("prev_door_control: ", self.prev_door_control.data)
            self.robot_door_control_pub.publish(self.door_control)
            self.prev_door_control.data = deepcopy(self.door_control.data)
            self.publish_pulse = True
            self.door_pulse_timer = rospy.Time.now()
            
        #! reset the doors control after 1 sec
        if self.publish_pulse and rospy.Time.now() - self.door_pulse_timer > rospy.Duration(1):
            
            self.pod_doors_control["front_right"],self.pod_doors_control["front_left"],self.pod_doors_control["back_right"],self.pod_doors_control["back_left"] = 0,0,0,0
            
            self.door_control.data = [self.pod_doors_control["top"] ,self.pod_doors_control["front_right"], self.pod_doors_control["front_left"], self.pod_doors_control["back_right"], self.pod_doors_control["back_left"]]
            self.publish_pulse = False

    def publish_emergency_cause(self):
        if rospy.Time.now() - self.prev_emergency_cause_time < rospy.Duration(self.emergency_check_time):
            return
        self.prev_emergency_cause_time = rospy.Time.now()

        #? update the emergency cause array
        self.emergency_cause_array.clear()
        self.emergency_cause_array = deepcopy(self.Robot_Feedback.getRobotEmergencyCauseArray())
        
        topic_error = self.ros_nodes_topics_check(self.robot_state)
        
        self.emergency_cause_array.extend(topic_error)
        
        if self.prev_emergency_cause_array == self.emergency_cause_array:
            return
        
        # todo in case of no emergency make it STAND_BY
        self.robot_state = self.old_robot_state if not self.emergency_cause_array else "EMERGENCY"
        
        self._emergency_cause_array_pub.publish(self.emergency_cause_array)
        self.prev_emergency_cause_array = deepcopy(self.emergency_cause_array)

    def publish_robot_state(self):

        if self.robot_state == self.prev_robot_state:
            return
        
        if self.robot_state != "EMERGENCY":
            self.old_robot_state = self.robot_state

        if self.robot_state != "KEY_OFF": # reset the timer
            self.elapsed_init_time = rospy.Time.now().to_sec()

        print("robot_state: ", self.robot_state)
        self.prev_robot_state = deepcopy(self.robot_state)
        self._robot_state_pub.publish(self.robot_state)


    
    def publish_teleop(self):
        self.teleoperator_command = self.Robot_Control.teleop_control()
        self.robot_command = self.Robot_Control.priority_control(self.auto_pilot_command, self.teleoperator_command )
        self.Robot_Control.get_robot_command(self.robot_command, self.robot_velocity_rpm, self.robot_steering, self.robot_emergency_brake)

        # ! emergency brake if the connection quality is less than 70
        self.robot_emergency_brake.data = True if (self.connection_quality < 70 or not self.websocket_connection) else self.robot_emergency_brake.data

        self.tele_operator_pub.     publish(self.teleoperator_command)
        self.velocity_pub.          publish(self.robot_velocity_rpm)
        self.steering_pub.          publish(self.robot_steering)
        
        if self.prev_robot_emergency_brake != self.robot_emergency_brake:
            self.emergency_brake_pub.   publish(self.robot_emergency_brake)
            self.prev_robot_emergency_brake = deepcopy(self.robot_emergency_brake)
        
        self.publish_doors_control()


    

    def publish_robot_operational_details(self):
        
        self.Robot_Feedback.updateDriveMode(self.robot_velocity_rpm.data)
        self.Robot_Feedback.updateDirectionLight(self.robot_steering.data)
        self.Robot_Feedback.updateDoorState()
        
        self.battery_capacity = self.Robot_Feedback.getBatteryCapacity()
        self.connection_quality = self.Robot_Feedback.getConnectionQuality()
        self.robot_operational_details = self.Robot_Feedback.getRobotDetails(self.robot_state, self.elapsed_init_time)
        
        self._robot_operational_details_pub.publish(self.robot_operational_details)
    
    def ros_nodes_topics_check(self, robot_state):
        
        if robot_state == "STAND_BY" or robot_state == "KEY_OFF" :
            # return
            pass
        
        error_nodes = []
        
        # ? CAN_INTERFACE must always be running
        self.can_check = self.check_node("/can_node")
        if not self.can_check:
            error_nodes.append("Can interface not running")
            
        # ? WebSocket must always be running
        self.websocket_connection = self.check_node("/rosbridge_websocket")
        if not self.websocket_connection:
            error_nodes.append("WebSocket not running")
        
         
        # todo if robot_state == "MAPPING" and self.mapping:        
        self.velodyne_check = self.check_node("/velodyne_nodelet_manager")
        
        if self.pcl_rate == 0:
            error_nodes.append("Lidar not publishing")
        
        if not self.velodyne_check:
            error_nodes.append("Velodyne node not running")
                
            
        return error_nodes
        
    def update(self):
        
        self.publish_robot_state()
        self.publish_health_check()
        self.publish_emergency_cause()
        
        if rospy.Time.now() - self.prev_publish_time > rospy.Duration(0.2):
            self.publish_robot_operational_details()
            self.prev_publish_time = rospy.Time.now()
            
            
            
            
        # ! for mapping it should be in the loop not in the service callback
        self.launch_map_server()
            
        self.publish_teleop()


if __name__ == "__main__":
    try:
        robot = Robot_Node()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            robot.update()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
