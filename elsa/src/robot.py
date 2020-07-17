from __future__ import division
import RPi.GPIO as gpio
import numpy as np
import time

class MotorTicks:

    def __init__(self):
        self.left = 0
        self.right = 0
        self.leftDeltaTicks = 0
        self.rightDeltaTicks = 0

    # def updateTicks(self):
    #     # self.left = 0
    #     # self.right = 0
    #     counterBL = np.uint64(0)
    #     counterBR = np.uint64(0)
    #
    #     buttonBL = int(0)
    #     buttonBR = int(0)
    #
    #     if int(gpio.input(12)) != int(buttonBR):
    #         buttonBR = int(gpio.input(12))
    #         counterBR += 1
    #     if int(gpio.input(7)) != int(buttonBL):
    #         buttonBL = int(gpio.input(7))
    #         counterBL += 1
    #
    #     self.left = counterBL
    #     self.right = counterBR



class MockRobot:
    """Implements a mock robot that dutifully executes its wheel
    speed commands exactly.
    """

    def __init__(self):
        self.leftSpeed = 0
        self.newLeftSpeed = 0
        self.rightSpeed = 0
        self.newRightSpeed = 0
        self.leftTicks = 0
        self.rightTicks = 0
        self.minTicks = -32768
        self.maxTicks = 32767
        # self.initialize_pins()

    # def initialize_pins(self):
    #
    #     gpio.setmode(gpio.BOARD)
    #     gpio.setup(7, gpio.IN, pull_up_down = gpio.PUD_UP)
    #     gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)


    def setSpeeds(self, left, right):
        self.newLeftSpeed = left
        self.newRightSpeed = right

    def updateRobot(self, dTime):
        self.leftTicks = self.addTicks(self.leftTicks, self.leftSpeed*dTime)
        self.rightTicks = self.addTicks(self.rightTicks, self.rightSpeed*dTime)
        self.leftSpeed = self.newLeftSpeed
        self.rightSpeed = self.newRightSpeed

    def getTicks(self):
        ticks = MotorTicks()
        ticks.updateTicks()
        ticks.left = self.leftTicks
        ticks.right = self.rightTicks
        return ticks

    def addTicks(self, ticks, deltaTicks):
        ticks += deltaTicks
        if ticks > self.maxTicks:
            return int(ticks - self.maxTicks + self.minTicks)
        elif ticks < self.minTicks:
            return int(ticks - self.minTicks + self.maxTicks)
        else:
            return int(ticks)
