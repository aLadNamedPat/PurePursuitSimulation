class Point:
    def __init__(self, xPos, yPos):
        self.x = xPos
        self.y = yPos
        self.vel = 0
    def setVel(self, vel):
        self.vel = vel
    def tupleForm(self):
        return (self.x, self.y)