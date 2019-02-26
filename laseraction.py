#!/usr/bin/env python

import roslib
import rospy
import math
import random

from sensor_msgs.msg import LaserScan

class LaserAction:

    left_laser = 1
    right_laser = 1
    center_laser = 1

    nearest_section = -1

    _const_sections_1 = None
    _const_sections_2 = None
    _sensor_data = None

    def __init__(self):
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_detection)

    def laser_detection(self, laser_scanner):

        self.left_laser = 1
        self.right_laser = 1
        self.center_laser = 1

        self._sensor_data = len(laser_scanner.ranges)
        self._const_sections_1 = self._sensor_data / 3
        self._const_sections_2 = self._const_sections_1 * 2

        for value in range (0, self._sensor_data):

            if 0.001 < laser_scanner.ranges[value] < 1.000:

                if 0 < value < self._const_sections_1:
                    self.right_laser = min(self.right_laser, laser_scanner.ranges[value])
                elif self._const_sections_1 < value < self._const_sections_2:
                    self.center_laser = min(self.center_laser, laser_scanner.ranges[value])
                elif self._const_sections_2 < value < self._sensor_data:
                    self.left_laser = min(self.left_laser, laser_scanner.ranges[value])

        #debugg laser

        rospy.loginfo("laser right: %f"% self.right_laser)
        rospy.loginfo("laser center: %f"% self.center_laser)
        rospy.loginfo("laser left: %f\n"% self.left_laser)

        self.nearest_section = min(self.left_laser, self.right_laser, self.center_laser)


if __name__ == '__main__':
    rospy.init_node('laser_action_subscriber', anonymous = True)
    laser_action = LaserAction()
    rospy.spin()
