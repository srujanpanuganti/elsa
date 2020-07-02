#! /usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from pose import Pose

import numpy as np
from math import pi,sin,cos,tan

class OdomPub:

    def __init__(self):

        self.lwheel_prev = int(0)
        self.rwheel_prev = int(0)
        self.lwheel_cur = int(0)
        self.rwheel_cur = int(0)

        self.leftTicks = 0
        self.rightTicks = 0

        self.x_prev = 0
        self.y_prev = 0
        self.x_cur = 0
        self.y_cur = 0

        self.theta_prev = 0     # in rad
        self.theta_cur = 0      # in rad

        self.lin_x_vel = 0
        self.lin_y_vel = 0

        self.yaw_vel = 0

        self.wheelRad = 0.0325 ##in mts
        self.gearRatio = 120/1
        self.ticksPerOneMotorRot = 8
        self.ticksPerOneWheelRot = self.gearRatio * self.ticksPerOneMotorRot  #960
        self.oneMeterToWheelRot = 1/(2 * pi * self.wheelRad)
        self.oneMeterToEncoderTicks = self.oneMeterToWheelRot * self.ticksPerOneWheelRot
        self.oneTickInMeters = 1/self.oneMeterToEncoderTicks
        self.wheelSeparation = 14.5 * 0.01 # metres
        self.pose = Pose()
        self.baseFrameID = '/base_link'
        self.odomFrameID = '/odom'


    def main(self):

        rospy.init_node('odom_pub')

        rospy.Subscriber('~lwheel_ticks', Int32, self.leftWheelCallback)
        rospy.Subscriber('~rwheel_ticks', Int32, self.rightWheelCallback)
        rospy.Subscriber("initialpose", PoseWithCovarianceStamped,
                         self.initial_pose)
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tfPub = TransformBroadcaster()

        self.nodeName = rospy.get_name()
        rospy.loginfo("{0} started".format(self.nodeName))

        self.rate = rospy.get_param('~rate', 10.0)
        self.timeout = rospy.get_param('~timeout', 0.5)

        rate = rospy.Rate(self.rate)
        self.prevTime = rospy.get_time()

        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


    def ticksToMeters(self, ticks):
        return ticks * self.oneTickInMeters

    def publish(self):

        deltaLeftWheel = self.lwheel_cur = self.lwheel_prev
        deltarightWheel = self.rwheel_cur = self.rwheel_prev

        currTime = rospy.get_time()

        timeElapsed = currTime - self.prevTime

        if deltaLeftWheel == deltarightWheel:
            #this means there's no turn
            distTravelledinTicks = deltarightWheel ## we get in ticks
            distTravelledinMeters = self.ticksToMeters(distTravelledinTicks)

            self.x_cur = self.x_prev + distTravelledinMeters * cos(self.theta_prev)
            self.y_cur = self.y_prev + distTravelledinMeters * sin(self.theta_prev)
            self.theta_cur = self.theta_prev
            theta_turned = 0

            self.lin_x_vel = (self.x_cur - self.x_prev)/ timeElapsed
            self.lin_y_vel = (self.y_cur - self.y_prev)/ timeElapsed
            self.yaw_vel =  theta_turned/timeElapsed

        else:
            ## this means there's a turn as well
            if deltaLeftWheel > deltarightWheel:  #turned right

                distTravelledinTicks = deltaLeftWheel ## we get in ticks
                distTravelledinMeters = self.ticksToMeters(distTravelledinTicks)

                theta_turned = distTravelledinMeters/self.wheelSeparation
                self.theta_cur = self.theta_prev - theta_turned

                ## finding center of curvature
                radCurv = self.wheelSeparation/2   #radius of curvature

                Cx = self.x_prev - radCurv * cos(self.theta_prev)
                Cy = self.y_prev - radCurv * sin(self.theta_prev)

                self.x_cur = Cx + radCurv * cos(self.theta_cur)
                self.y_cur = Cy + radCurv * sin(self.theta_cur)

                self.lin_x_vel = (self.x_cur - self.x_prev)/ timeElapsed
                self.lin_y_vel = (self.y_cur - self.y_prev)/ timeElapsed
                self.yaw_vel = theta_turned/timeElapsed


            elif deltarightWheel > deltaLeftWheel:  #turned left

                distTravelledinTicks = deltarightWheel ## we get in ticks
                distTravelledinMeters = self.ticksToMeters(distTravelledinTicks)

                theta_turned = distTravelledinMeters/self.wheelSeparation
                self.theta_cur = self.theta_prev + theta_turned

                ## finding center of curvature
                radCurv = self.wheelSeparation/2   #radius of curvature

                Cx = self.x_prev - radCurv * cos(self.theta_prev)
                Cy = self.y_prev - radCurv * sin(self.theta_prev)

                self.x_cur = Cx + radCurv * cos(self.theta_cur)
                self.y_cur = Cy + radCurv * sin(self.theta_cur)

                self.lin_x_vel = (self.x_cur - self.x_prev)/ timeElapsed
                self.lin_y_vel = (self.y_cur - self.y_prev)/ timeElapsed
                self.yaw_vel = theta_turned/timeElapsed


        self.pose.x = self.x_cur
        self.pose.y = self.y_cur
        self.pose.theta = self.theta_cur

        self.pose.xVel = self.lin_x_vel
        self.pose.yVel = self.lin_y_vel
        self.pose.thetaVel = self.yaw_vel


        q = quaternion_from_euler(0, 0, self.pose.theta)
        self.tfPub.sendTransform(
            (self.pose.x, self.pose.y, 0),
            (q[0], q[1], q[2], q[3]),
            currTime,
            self.baseFrameID,
            self.odomFrameID
        )

        odom = Odometry()
        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = self.odomFrameID
        odom.child_frame_id = self.baseFrameID
        odom.pose.pose.position.x = self.pose.x
        odom.pose.pose.position.y = self.pose.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.pose.xVel
        odom.twist.twist.angular.z = self.pose.thetaVel
        self.odomPub.publish(odom)

        self.x_prev = self.x_cur
        self.y_prev = self.y_cur
        self.prevTime = currTime
        self.theta_prev = self.theta_cur

    def leftWheelCallback(self, lwheelticks):
        self.leftTicks = lwheelticks.data
        self.lwheel_cur = self.leftTicks

    def rightWheelCallback(self, rwheelticks):
        self.rightTicks = rwheelticks.data
        self.lwheel_cur = self.rightTicks


    def initial_pose(self, msg):
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # pose = Pose()
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta = yaw

        self.x_prev = self.pose.x
        self.y_prev = self.pose.y
        self.theta_prev = self.pose.theta     # in rad

        rospy.loginfo('Setting initial pose to %s', self.pose)
