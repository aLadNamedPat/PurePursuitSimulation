import pygame
from Env import Env
from Robot import Robot
from BezierCurves import BezierCurve
from Point import Point
pygame.init()

environment = Env((432, 432), "Field.png")
robot = Robot(100, 100, 36, "RobotImg.png", orientation=180 )
P0 = Point(100, 100)
P1 = Point(350, 300)     
P2 = Point(300, 300)
P3 = Point(250, 350)

# bezierCurve = BezierCurve(P0, P1, P2, P3, 20)
robot.buildBezierCurve(P0, P1, P2, P3)

running = True
x = 0
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    pygame.display.update()
    environment.map.fill(environment.white)
    environment.draw((432, 432))
    robot.bezierCurve.draw(environment.map, 3)
    robot.basicPurePursuit(20)
    robot.draw(environment.map)
# P5 = Point(125, 90)
# robot.lookAheadDistance = 25
# robot.calculateCurvature(P5)
# print(robot.degrees)
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#     pygame.display.update()
#     environment.map.fill(environment.white)
#     environment.draw((432, 432))
#     robot.draw(environment.map)
    
