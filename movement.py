#!/usr/bin/env python

import roslib
import rospy
import math
import random

from geometry_msgs.msg import Twist

from bumperaction import BumperAction


class Movement:

    _forward_speed = 0.30
    _rate_value = 60

    def __init__(self, rate_pub = _rate_value ,vel_forward = _forward_speed):
        self._rate_value = rate_pub
        self._forward_speed = vel_forward

        self._move_publish = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.bumper_detection = BumperAction()


    def move_forward(self):

        _rate_pub_forward = rospy.Rate(self._rate_value)
        _move_forward_msg = Twist()

        _debug_str = "No obstacles in front! Going forward %s" % rospy.get_time()
        _debug_str_halt = "No speed! Something halt me."

        _move_forward_msg.linear.x = self._forward_speed
        _move_forward_msg.linear.y = 0
        _move_forward_msg.linear.z = 0

        _move_forward_msg.angular.x = 0
        _move_forward_msg.angular.y = 0
        _move_forward_msg.angular.z = 0

        self._move_publish.publish(_move_forward_msg)

        if self.bumper_detection.get_bumper_collision():
            self.halt_movement()

        _rate_pub_forward.sleep()

        if self._forward_speed == 0:
            rospy.loginfo(_debug_str_halt)
        else:
            rospy.loginfo(_debug_str)

    def turn_around(self):

        _move_around_msg = Twist()
        _rate_pub_around = rospy.Rate(self._rate_value)

        _debug_str = "Escaping! Turning around! %s" % rospy.get_time()

        _move_around_msg.linear.x = 0
        _move_around_msg.linear.y = 0
        _move_around_msg.linear.z = 0
        _move_around_msg.angular.x = 0
        _move_around_msg.angular.y = 0

        turn_angle = random.randrange(150, 210)

        _move_around_msg.angular.z = math.radians(turn_angle)

        rospy.loginfo(_debug_str)

        self.stop_motion()

        for _ in range(0, 5):
            self._move_publish.publish(_move_around_msg)
            _rate_pub_around.sleep()

    def turn_to_sides(self, direction):

        _turn_sides = Twist()

        _rate_turn = rospy.Rate(10)

        turn_angle_1eft = random.randrange(-90, -10)
        turn_angle_right = random.randrange(10, 90)

        #while _currentTime < _stopTime:
        _turn_sides.linear.x = 0
        _turn_sides.linear.y = 0
        _turn_sides.linear.z = 0
        _turn_sides.angular.x = 0
        _turn_sides.angular.y = 0

        if direction == "LEFT":
            _turn_sides.angular.z = math.radians(turn_angle_right)
        elif direction == "RIGHT":
            _turn_sides.angular.z = math.radians(turn_angle_1eft)

        #for _ in range(10):
        self._move_publish.publish(_turn_sides)
        _rate_turn.sleep()

    def turn_randomly(self):

        _rate_pub_turn = rospy.Rate(self._rate_value)
        _turn_random_msg = Twist()

        _debug_str = "turn random %s" % rospy.get_time()

        _turn_random_msg.linear.x = 0
        _turn_random_msg.linear.y = 0
        _turn_random_msg.linear.z = 0

        _turn_random_msg.angular.x = 0
        _turn_random_msg.angular.y = 0

        turn_angle = random.randrange(-20, 20)
        _turn_random_msg.angular.z = math.radians(turn_angle)

        for _ in range(0, 60):

            if self.bumper_detection.get_bumper_collision():
                self.halt_movement()

            self._move_publish.publish(_turn_random_msg)
            _rate_pub_turn.sleep()

        rospy.loginfo(_debug_str)

    def forward_and_turn(self):

        _init_time = rospy.Time.now().to_sec()

        _distance = 0
        _distance_to_move = 0.35

        while _distance < _distance_to_move:
            self.move_forward()
            _current_time = rospy.Time.now().to_sec()
            _distance = self._forward_speed * (_current_time -_init_time)

        self.stop_motion()
        rospy.loginfo("0.3048 forward, TIME TO TURN")
        self.turn_randomly()
        self.stop_motion()

    def stop_motion(self):
        _rate_stop = rospy.Rate(self._rate_value)
        _stop_msg = Twist()

        _debug_str = "Technical stop %s" % rospy.get_time()

        _stop_msg.linear.x = 0
        _stop_msg.linear.y = 0
        _stop_msg.linear.z = 0

        _stop_msg.angular.x = 0
        _stop_msg.angular.y = 0
        _stop_msg.angular.z = 0

        if self.bumper_detection.get_bumper_collision():
            self.halt_movement()

        rospy.loginfo(_debug_str)

        for _ in range(0, 60):
            self._move_publish.publish(_stop_msg)
        _rate_stop.sleep()

    def halt_movement(self):

        _rate_pub_halt = rospy.Rate(self._rate_value)
        _halt_msg = Twist()

        _debug_str = "I get hit. BE CAREFUL. Halt. %s" % rospy.get_time()
        _debug_halt = "Shutdown. Something with me!"

        self._forward_speed = 0

        _halt_msg.linear.x = self._forward_speed
        _halt_msg.linear.y = 0
        _halt_msg.linear.z = 0

        _halt_msg.angular.x = 0
        _halt_msg.angular.y = 0
        _halt_msg.angular.z = 0

        rospy.loginfo(_debug_str)
        self._move_publish.publish(_halt_msg)
        _rate_pub_halt.sleep()

        rospy.signal_shutdown(_debug_halt)

if __name__ == '__main__':
    rospy.init_node('movement_publisher', anonymous = True)
    move = Movement()
    rospy.spin()
