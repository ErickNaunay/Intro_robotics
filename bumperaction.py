#!/usr/bin/env python

import roslib
import rospy

from kobuki_msgs.msg import BumperEvent

class BumperAction:

    _bumper_collision = False

    def __init__(self):
        self.bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumper_collision)

    def bumper_collision(self, data):

        if data.bumper == 0 or data.bumper == 1 or data.bumper == 2 or data.state == BumperEvent.PRESSED or data.state == BumperEvent.RELEASED:
            self._bumper_collision = True
        else:
            self._bumper_collision = False
        rospy.loginfo("%d"%data.bumper)

    def get_bumper_collision(self):
        return self._bumper_collision

if __name__ == '__main__':
    rospy.init_node('bumper_action_subscriber', anonymous = True)
    bumper_Action = BumperAction()
    rospy.spin()
