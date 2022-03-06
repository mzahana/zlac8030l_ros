#!/usr/bin/env python3

"""
ROS driver for the ZLAC8030L motor controller
Reuquires: ZALAC8030L_CAN_controller, https://github.com/mzahana/ZLAC8030L_CAN_controller
"""

from doctest import Example
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ZLAC8030L_CAN_controller.canopen_controller import MotorController

class Driver:
    def __init__(self) -> None:
        self._can_channel = rospy.get_param("~can_channel", "can0")
        self._bus_type = rospy.get_param("~bus_type", "socketcan")
        self._bitrate = rospy.get_param("~bitrate", 500000)
        self._eds_file = rospy.get_param("~eds_file","")
        self._wheel_ids = rospy.get_param("~wheel_ids", []) # TODO needs checking
        
        self._wheel_radius = rospy.get_param("~wheel_radius", 0.194)

        try:
            self._network = MotorController(channel=self._can_channel, bustype=self._bus_type, bitrate=self._bitrate, node_ids=None, debug=True, eds_file=self._eds_file)
        except Exception as e:
            rospy.logerr("Could not create CAN network object. Error: %s", e)
            exit(10)

        # ------------------- Subscribers ----------------#
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)

    def cmdVelCallback(self, msg):
        pass

def main():
    pass

if __name__ == "__main__":
    main()