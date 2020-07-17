from .pose import Pose
from .encoder import Encoder
from math import cos, sin, pi

class Odometry:

    def __init__(self):

        self.BL_encoder = Encoder()
        self.BR_encoder = Encoder()

        self.pose = Pose()
        self.lastTime = 0


    def setWheelSeparation(self, separation):
        self.wheelSeparation = separation

    def setTicksPerMeter(self, ticks):
        self.ticksPerMeter = ticks

    def setEncoderRange(self, low, high):
        self.BL_encoder.setRange(low, high)
        self.BR_encoder.setRange(low, high)

    def setTime(self, newTime):
        self.lastTime = newTime

    def updateLeftWheel(self, newCount):
        self.BL_encoder.update(newCount)

    def updateRightWheel(self, newCount):
        self.BR_encoder.update(newCount)


    def updatePose(self, newTime):
        """Updates the pose based on the accumulated encoder ticks
        of the two wheels. See https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
        for details.
        """
        leftTravel = self.BL_encoder.getDelta() / self.ticksPerMeter
        rightTravel = self.BR_encoder.getDelta() / self.ticksPerMeter
        deltaTime = newTime - self.lastTime

        deltaTravel = (rightTravel + leftTravel) / 2
        deltaTheta = (rightTravel - leftTravel) / self.wheelSeparation

        if rightTravel == leftTravel:
            deltaX = leftTravel*cos(self.pose.theta)
            deltaY = leftTravel*sin(self.pose.theta)
        else:
            radius = deltaTravel / deltaTheta

            # Find the instantaneous center of curvature (ICC).
            iccX = self.pose.x - radius*sin(self.pose.theta)
            iccY = self.pose.y + radius*cos(self.pose.theta)

            deltaX = cos(deltaTheta)*(self.pose.x - iccX) \
                - sin(deltaTheta)*(self.pose.y - iccY) \
                + iccX - self.pose.x

            deltaY = sin(deltaTheta)*(self.pose.x - iccX) \
                + cos(deltaTheta)*(self.pose.y - iccY) \
                + iccY - self.pose.y

        self.pose.x += deltaX
        self.pose.y += deltaY
        self.pose.theta = (self.pose.theta + deltaTheta) % (2*pi)
        self.pose.xVel = deltaTravel / deltaTime if deltaTime > 0 else 0.
        self.pose.yVel = 0
        self.pose.thetaVel = deltaTheta / deltaTime if deltaTime > 0 else 0.

        self.lastTime = newTime



    def getPose(self):
        return self.pose

    def setPose(self, newPose):
        self.pose = newPose
