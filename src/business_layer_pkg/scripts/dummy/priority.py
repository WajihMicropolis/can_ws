#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


class priorityNode:
    def __init__(self):
        rospy.init_node("priority_control", anonymous=True)
        self.navigation_sub = rospy.Subscriber("cmd_vel_navigation", Twist, self.navigation_cb)
        self.navigation_sub = rospy.Subscriber("tele_operator", Twist, self.tele_operator_cb)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)


    def navigation_cb(self, msg: Twist):
        None

    def tele_operator_cb(self,msg: Twist):
        None
        
        
        
if __name__ == "__main__":
    try:
        robot = priorityNode()

        while not rospy.is_shutdown():
            # robot.keyboard_control(_print=True)
            # robot.priority_control()
            rospy.sleep(0.05)

    except rospy.ROSInterruptException:
        pass
