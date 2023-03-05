import pygame

class Env:
    def __init__(self, dimensions, img):
        self.black = (0,0,0)
        self.white = (255, 255, 255)
        self.height = dimensions[0]
        self.width = dimensions[1]  
        self.map = pygame.display.set_mode((self.width + 50, self.height + 50))
        self.img = pygame.image.load(img)

    def draw(self, dimensions):
        self.img = pygame.transform.scale(self.img, (dimensions[0], dimensions[1]))
        self.map.blit(self.img, (25,25))
