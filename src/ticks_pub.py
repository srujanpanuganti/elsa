#! /usr/bin/env python
from __future__ import division

import rospy
from math import pi, asin
# from geometry_msgs.msg import Twist, Pose
# from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import numpy as np
import RPi.GPIO as gpio


class TickPublisher:

    def __init__(self):

        self.counterBL = np.uint64(0)
        self.counterBR = np.uint64(0)

        self.buttonBL = int(0)
        self.buttonBR = int(0)

        self.left = self.counterBL
        self.right = self.counterBR

        self.initialize_pins()

    def initialize_pins(self):

        gpio.setmode(gpio.BOARD)
        gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
        gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

    def main(self):

        rospy.init_node('ticks_pub')
        self.leftPub = rospy.Publisher('~lwheel_ticks',
                                       Int32, queue_size=10)
        self.rightPub = rospy.Publisher('~rwheel_ticks',
                                        Int32, queue_size=10)

        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))


        self.rate = rospy.get_param('~rate', 10.0)
        self.timeout = rospy.get_param('~timeout', 0.5)

        rate = rospy.Rate(self.rate)
        self.lastTime = rospy.get_time()
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


    def publish(self):

        if int(gpio.input(12)) != int(self.buttonBR):
            self.buttonBR = int(gpio.input(12))
            self.counterBR += 1
        if int(gpio.input(7)) != int(self.buttonBL):
            self.buttonBL = int(gpio.input(7))
            self.counterBL += 1


        self.left = self.counterBL
        self.right = self.counterBR

        print(self.left, self.right)

        self.leftPub.publish(self.left)
        self.rightPub.publish(self.right)


if __name__ == '__main__':
    try:
        node = TickPublisher()
        node.main()
    except rospy.ROSInterruptException:
        pass
