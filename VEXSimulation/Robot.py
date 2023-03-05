import pygame
import numpy as np
import math
from Point import Point
from BezierCurves import BezierCurve
class Robot:
    def __init__(self, xInitial, yInitial, trackWidth, robotImg, orientation = 180):
        self.x = xInitial
        self.y = yInitial
        self.degrees = orientation
        self.trackWidth = trackWidth
        self.leftWheelVel = 0
        self.rightWheelVel = 0
        self.leftWheelAccel = 0
        self.rightWheelAccel = 0
        self.angvel = 0
        self.turnRad = 0
        self.img = pygame.image.load(robotImg)
        self.img = pygame.transform.scale(self.img, (trackWidth, 54))
        self.rotated = pygame.transform.rotate(self.img, self.degrees)
        self.rect = self.rotated.get_rect(center = (self.x, self.y))
        self.maxVel =  100
        self.maxAccel = 100
        self.lastPos = 0
        self.lastLeftTargetVel = 0
        self.lastRightTargetVel = 0
    def setVel(self, leftWheel, rightWheel):
        self.leftWheelVel = leftWheel
        self.rightWheelVel = rightWheel
    
    def move(self):
        self.timeConstant = 0.5
        # self.leftWheelVel += self.leftWheelAccel
        # self.rightWheelVel += self.rightWheelAccel
        # if (self.leftWheelVel > self.maxVel):
        #     self.leftWheelVel = self.maxVel
        # if (self.rightWheelVel > self.maxVel):
        #     self.rightWheelVel = self.maxVel

        if (self.leftWheelVel == self.rightWheelVel):
            self.x -= math.sin(self.degrees / 180 * math.pi) * self.leftWheelVel * self.timeConstant
            self.y -= math.cos(self.degrees / 180 * math.pi) * self.leftWheelVel * self.timeConstant 

        else:
            self.angvel = (self.rightWheelVel - self.leftWheelVel) / self.trackWidth
            self.turnRad = -self.angvel * self.timeConstant
            self.ICCcenter = (self.x - (self.trackWidth / 2 + self.leftWheelVel * self.trackWidth 
                                        / (self.rightWheelVel - self.leftWheelVel)) 
                                        * math.cos(self.degrees / 180 * math.pi), 
                              self.y - (self.trackWidth / 2 + self.leftWheelVel * self.trackWidth 
                                        / (self.rightWheelVel - self.leftWheelVel)) 
                                        * math.sin(self.degrees / 180 * math.pi))
            self.xPrime = self.x - self.ICCcenter[0]
            self.yPrime = self.y - self.ICCcenter[1]
            self.xPrime = self.xPrime * math.cos(self.turnRad) - self.yPrime * math.sin(self.turnRad)
            self.yPrime = self.xPrime * math.sin(self.turnRad) + self.yPrime * math.cos(self.turnRad)
            self.x = self.xPrime + self.ICCcenter[0]
            self.y = self.yPrime + self.ICCcenter[1]
            self.degrees += self.turnRad * 180 / math.pi
            if (self.degrees < 0):
                self.degrees += 360
            elif (self.degrees > 360):
                self.degrees -= 360
            self.rotated = pygame.transform.rotate(self.img, -self.degrees)
        self.rect = self.rotated.get_rect(center = (self.x, self.y))

    def buildBezierCurve(self, P0, P1, P2, P3):
        self.bezierCurve = BezierCurve(P0, P1, P2, P3, self.maxAccel)
        self.savedPathPoints = self.bezierCurve.pointStorage
            
    def sgn(self, val):
        if (val >= 0):
            return 1
        else:
            return -1
        

    def findNextPoint(self, lookAheadDistance):
        goalPosition = self.savedPathPoints[self.lastPos]
        for i in range(self.lastPos, len(self.savedPathPoints) - 1):
            P0 = self.savedPathPoints[i]
            P1 = self.savedPathPoints[i + 1]

            x1 = P0.x
            y1 = P0.y
            x2 = P1.x
            y2 = P1.y

            x1_offset = x1 - self.x
            y1_offset = y1 - self.y
            x2_offset = x2 - self.x
            y2_offset = y2 - self.y

            dx = x2 - x1
            dy = y2 - y1
            
            dr = math.sqrt(dx**2 + dy ** 2)
            D = x1_offset * y2_offset - x2_offset * y1_offset
            discriminant = (lookAheadDistance** 2) * (dr** 2) - D**2

            if discriminant >= 0:
                sol_x1 = (D * dy + self.sgn(dy) * dx * math.sqrt(discriminant)) / (dr*dr)
                sol_x2 = (D * dy - self.sgn(dy) * dx * math.sqrt(discriminant)) / (dr*dr)
                sol_y1 = (-D * dx + abs(dy) * math.sqrt(discriminant)) / (dr*dr)
                sol_y2 = (-D * dx - abs(dy) * math.sqrt(discriminant)) / (dr*dr)

                sol1 = Point(sol_x1 + self.x, sol_y1 + self.y)
                sol2 = Point(sol_x2 + self.x, sol_y2 + self.y)

                minX = min(x1, x2)
                maxX = max(x1, x2)
                minY = min(y1, y2)
                maxY = max(y1, y2)

                if ((minX <= sol1.x and sol1.x <= maxX and minY <= sol1.y and sol1.y <= maxY) or 
                    (minX <= sol2.x and sol2.x <= maxX and minY <= sol2.y and sol2.y <= maxY)):
                        if ((minX <= sol1.x and sol1.x <= maxX and minY <= sol1.y and sol1.y <= maxY) and 
                        (minX <= sol2.x and sol2.x <= maxX and minY <= sol2.y and sol2.y <= maxY)):
                            if (np.linalg.norm(np.array(self.savedPathPoints[i + 1].tupleForm()) - np.array(sol1.tupleForm())) < 
                                np.linalg.norm(np.array(self.savedPathPoints[i + 1].tupleForm()) - np.array(sol2.tupleForm()))):
                                goalPosition = sol1
                            else:
                                goalPosition = sol2
                        else:
                            if (minX <= sol2.x and sol2.x <= maxX and minY <= sol2.y and sol2.y <= maxY):
                                goalPosition = sol2
                            else:
                                goalPosition = sol1
                        if (np.linalg.norm(np.array(goalPosition.tupleForm()) - np.array(self.savedPathPoints[i + 1].tupleForm())) < 
                            np.linalg.norm(np.array(self.savedPathPoints[i + 1].tupleForm()) - np.array((self.x, self.y)))): 
                            self.lastPos = i 
                            break
                        else:
                            self.lastPos = i + 1
                else:
                    goalPosition = Point(self.savedPathPoints[self.lastPos].tupleForm()[0] , self.savedPathPoints[self.lastPos].tupleForm()[1])
        return self.savedPathPoints[self.lastPos]
    
    # def calculateCurvature(self, P0):
    #     P0magnitude = math.sqrt(P0.x ** 2 + P0.y ** 2)
    #     if (P0.x < self.x):
    #         if (P0.y < self.y):
    #             savedAngle = 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x - self.x))
    #         else:
    #             savedAngle = 360 - 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x - self.x))
    #     else:
    #         if (P0.y < self.y):
    #             savedAngle = 180 - 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x - self.x))
    #         else:
    #             savedAngle = 180 + 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x - self.x))
    #     savedAngle -= self.degrees
    #     if (savedAngle < 0):
    #         savedAngle += 360
    #     elif (savedAngle > 360):
    #         savedAngle -= 360
    #     P1 = Point(P0magnitude * math.cos(math.pi / 180 * savedAngle),
    #                P0magnitude * math.sin(math.pi / 180 * savedAngle))
    #     distanceDx = P1.x - self.x
    #     curvature = 2 * distanceDx / (self.lookAheadDistance**2)
    #     return curvature
    
    def calculateCurvature(self, P0):
        # if (P0.x < self.x):
        #     if (P0.y < self.y):
        #         savedAngle = 180 / math.pi * math.atan((self.x - P0.x) / (self.y - P0.y))
        #     else:
        #         savedAngle = 180 - 180 / math.pi * math.atan(abs(self.x - P0.x) / abs(self.y - P0.y))
        # else:
        #     if (P0.y > self.y):
        #         savedAngle = 180 + 180 / math.pi * math.atan(abs(self.x - P0.x) / abs(self.y - P0.y))
        #     else:
        #         savedAngle = 360 - 180 / math.pi * math.atan(abs(self.x - P0.x) / abs(self.y - P0.y))
        if (P0.x < self.x):
            if (P0.y > self.y):
                savedAngle = 180 / math.pi * math.atan((self.x - P0.x) / (self.y - P0.y))
            else:
                savedAngle = 180 - 180 / math.pi * math.atan(abs(self.x - P0.x) / abs(self.y - P0.y))
        else:
            if (P0.y > self.y):
                savedAngle = 180 + 180 / math.pi * math.atan(abs(self.x - P0.x) / abs(self.y - P0.y))
            else:
                savedAngle = 360 - 180 / math.pi * math.atan(abs(self.x - P0.x) / abs(self.y - P0.y))

        if (savedAngle > self.degrees):
            newAngle = savedAngle - self.degrees
            # print("This is the saved angle: " + str(savedAngle))
            # print("This is the robot measure: " + str(self.degrees))
            if (savedAngle - self.degrees > 180): #turning left needed
                side = 1
            else: 
                side = -1
        else:
            newAngle = self.degrees - savedAngle
            if (self.degrees - savedAngle > 180):
                side = -1
            else:  
                side = 1
        print("New Angle" + str(newAngle * side))
        # print(newAngle)
        # if (side < 1):
        #     print("right")
        # else: 
        #     print("left")
        xVal = math.sin(math.pi / 180 * newAngle) * self.lookAheadDistance 
        curvature = 2 * xVal / self.lookAheadDistance ** 2
        return curvature * side

    # def calculateCurvature2(self, P0):
    #     savedAngle = math.atan((P0.y - self.y)/(P0.x - self.x))
    #     a = - math.tan(savedAngle)
    #     b = 1
    #     c = math.tan(savedAngle) * self.x - self.y  
    #     d = abs(a * P0.x + b * P0.y + c) / math.sqrt(a ** 2 + b ** 2)
    #     return 2 * d  / self.lookAheadDistance ** 2

    def calculateAnglularError(self, P0):
        degrees = self.degrees  
        if (P0.x > self.x):
            if (P0.y > self.y):
                savedAngle = 270 - 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x  - self.x))
            else:
                savedAngle = 270 + 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x  - self.x))
        else:
            if (P0.y > self.y):
                savedAngle = 90 + 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x  - self.x))
            else:
                savedAngle = 90 - 180 / math.pi * math.atan(abs(P0.y - self.y) / abs(P0.x  - self.x))
        savedAngle -= degrees
        if savedAngle > 180:
            savedAngle = -360 + savedAngle
        elif (savedAngle < -180):
            savedAngle = 360 + savedAngle
        print(savedAngle)
        return savedAngle


    def basicPurePursuit(self, lookAheadDistance):
        self.lookAheadDistance = lookAheadDistance
        P0 = self.findNextPoint(lookAheadDistance)
        kA = 0.05
        kV = 2
        translationalVelocity = kV * self.lookAheadDistance
        angularVelocity = self.calculateAnglularError(P0) * kA
        self.setVel(translationalVelocity - angularVelocity, translationalVelocity + angularVelocity)
        self.move()

    def followPath(self, lookAheadDistance):
        self.lookAheadDistance = lookAheadDistance
        P0 = self.findNextPoint(lookAheadDistance)
        pointCurvature = self.bezierCurve.curvatureAtPoint(self.lastPos / 10000)
        # print("Point Curvature: " + str(self.calculateCurvature(P0)))
        P0.setVel(min(P0.vel, 2 / pointCurvature))

        kV = 1 / (self.maxVel)
        kA = 0.002
        kP = 0.001

        print ("Curvature Calculation:" + str(self.calculateCurvature(P0)))

        self.leftTargetVel = P0.vel * (2 + self.calculateCurvature(P0) * self.trackWidth) / 2
        self.rightTargetVel = P0.vel * (2 - self.calculateCurvature(P0) * self.trackWidth) / 2

        # print("VEL" + str(pointCurvature))
        # print(P0.vel)
        # print("rightSide" + str(self.rightTargetVel))
        self.rightTargetAccel = self.rightTargetVel - self.lastRightTargetVel
        self.leftTargetAccel = self.leftTargetVel - self.lastLeftTargetVel
        if (self.rightTargetAccel < -self.maxAccel):
            self.rightTargetAccel = -self.maxAccel
        if (self.leftTargetAccel < -self.maxAccel):
            self.leftTargetAccel = -self.maxAccel
        
        # self.FFL = kV * self.leftTargetVel + kA * self.leftTargetAccel
        # self.FBL = kP * (self.leftTargetVel - self.leftWheelVel)
        # self.FFR = kV * self.rightTargetVel + kA * self.rightTargetAccel
        # self.FBR = kP * (self.rightTargetVel - self.rightWheelVel)
        
        self.FFL = kV * self.leftTargetVel
        self.FFR = kV * self.rightTargetVel

        # print("VALSL:" + str(self.leftTargetVel) + ", " + str(self.leftTargetAccel) + " , " + str(self.leftTargetVel - self.leftWheelVel))
        # print("VALSR:" + str(self.rightTargetVel) + ", " + str(self.rightTargetAccel) + ", " + str(self.rightTargetVel - self.rightWheelVel))
        self.lastLeftTargetVel = self.leftTargetVel
        self.lastRightTargetVel = self.rightTargetVel


        # print("x" + str(self.x) + "y" + str(self.y))
        # print("lastPoint" + str(self.lastPos))
        # print(np.linalg.norm(np.array(self.savedPathPoints[self.lastPos].tupleForm()) - np.array((self.x, self.y))))
        # print("leftSide" + str(self.FFL + self.FBL))
        # print("rightSide" + str(self.FFR + self.FBR))
        # self.setVel(self.FFL + self.FBL, self.FFR + self.FBR)
        self.setVel(self.FFL, self.FFR)
        self.move()

    def draw(self, map):
        map.blit(self.rotated, self.rect)
        

    
