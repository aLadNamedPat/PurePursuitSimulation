from Point import Point
import pygame
import numpy as np
import math
class BezierCurve:
    def __init__(self, P0, P1, P2, P3, maxAccel):
        self.P0 = P0
        self.P1 = P1
        self.P2 = P2
        self.P3 = P3
        self.pointStorage = []
        self.maxAccel = maxAccel
        self.buildPath()

    def lerp(self, P0, P1, t):
        newPoint = Point(P0.x + (P1.x - P0.x) * t, P0.y + (P1.y - P0.y) * t)
        return newPoint
    
    def calculatePoint(self, i):
        P0P1 = self.lerp(self.P0, self.P1, i)
        P1P2 = self.lerp(self.P1, self.P2, i)
        P2P3 = self.lerp(self.P2, self.P3, i)
        A = self.lerp(P0P1, P1P2, i)
        B = self.lerp(P1P2, P2P3, i)
        C = self.lerp(A, B, i)
        return C
    
    def firstDerivative(self, i):
        firstDerivativeX = 3 * self.P1.x - 3 * self.P0.x + 6 * self.P0.x * i + 6 * self.P2.x * i - 12 * self.P1.x * i - 3 * self.P0.x * i**2 + 9 * self.P1.x * i**2 - 9 * self.P2.x * i**2 + 3 * self.P3.x * i**2
        firstDerivativeY = 3 * self.P1.y - 3 * self.P0.y + 6 * self.P0.y * i + 6 * self.P2.y * i - 12 * self.P1.y * i - 3 * self.P0.y * i**2 + 9 * self.P1.y * i**2 - 9 * self.P2.y * i**2 + 3 * self.P3.y * i**2
        return (firstDerivativeX, firstDerivativeY)
    
    def secondDerivative(self, i):
        secondDerivativeX = 6 * self.P0.x + 6 * self.P2.x - 12 * self.P1.x - 6 * self.P0.x * i + 18 * self.P1.x * i - 18 * self.P2.x * i + 6 * self.P3.x * i
        secondDerivativeY = 6 * self.P0.y + 6 * self.P2.y - 12 * self.P1.y - 6 * self.P0.y * i + 18 * self.P1.y * i - 18 * self.P2.y * i + 6 * self.P3.y * i
        return (secondDerivativeX, secondDerivativeY)
    
    def curvatureAtPoint(self, i):
        numerator = self.firstDerivative(i)[0] * self.secondDerivative(i)[1] - self.firstDerivative(i)[1] * self.secondDerivative(i)[0]
        denominator = math.sqrt(self.firstDerivative(i)[0] ** 2  + self.firstDerivative(i)[1] ** 2) ** 3
        curvature = abs(numerator / denominator)
        return curvature
    
    def curvatureAtPoint2(self, i):
        k1 = 0.5 * (self.pointStorage[i].x ** 2 + self.pointStorage[i].y ** 2 - self.pointStorage[i + 1].x**2 - self.pointStorage[i + 1].y ** 2) / (self.pointStorage[i].x - self.pointStorage[i + 1].x)
        k2 = (self.pointStorage[i].y - self.pointStorage[i + 1].y) / ((self.pointStorage[i].x) - self.pointStorage[i + 1].x)
        b = 0.5 * (self.pointStorage[i + 1].x ** 2 - 2 * self.pointStorage[i + 1].x * k1 + self.pointStorage[i + 1].y ** 2 - self.pointStorage[i + 2].x ** 2 + 2 * self.pointStorage[i + 2].x * k1 - self.pointStorage[i + 2].y ** 2) / (self.pointStorage[i + 2].x * k2 - self.pointStorage[i + 2].y + self.pointStorage[i + 1].y - self.pointStorage[i + 1].x * k2)
        a = k1 - k2 * b
        r = math.sqrt((self.pointStorage[i].x - a) ** 2 + (self.pointStorage[i].y - b) ** 2)
        curvature = 1 / r
        return curvature
    
    def buildPath(self):
        for i in range(1, 10000):
            newPoint = self.calculatePoint(i/10000)
            self.pointStorage.append(newPoint)
        oldVel = 0
        maxVel = 100
        distanceBetween = 0
        for t in range(len(self.pointStorage) - 1, 0, -1):
            distanceBetween = np.linalg.norm(
                np.array(self.pointStorage[t].tupleForm()) 
                - np.array(self.pointStorage[t - 1].tupleForm())
            )
            self.pointStorage[t].setVel(oldVel)
            oldVel = min(maxVel, math.sqrt(oldVel**2 + 2 * distanceBetween * self.maxAccel))

    def draw(self, map, runWidth):
        for i in range(len(self.pointStorage) - 1):
            pygame.draw.line(map, (0,0,0), self.pointStorage[i].tupleForm(), self.pointStorage[i + 1].tupleForm(), width = runWidth)