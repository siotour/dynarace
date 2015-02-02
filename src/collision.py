import math, sys, pygame
from pygame.math import *

class BoundingBox:
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def __init__(self, rect, theta = 0):
        a = pygame.math.Vector2(rect.left, rect.top)
        b = pygame.math.Vector2(rect.right, rect.top)
        c = pygame.math.Vector2(rect.right, rect.bottom)
        d = pygame.math.Vector2(rect.left, rect.bottom)



        self.a = (a - rect.center).rotate(theta) + rect.center
        self.b = (b - rect.center).rotate(theta) + rect.center
        self.c = (c - rect.center).rotate(theta) + rect.center
        self.d = (d - rect.center).rotate(theta) + rect.center

        #print(rect, " becomes ", self.__str__())

    def __str__(self):
        return "{}, {}, {}, {}".format(self.a, self.b, self.c, self.d)


# Returns true if p1 and p2 are on the same side of the line through a and b.
def sameSide(p1, p2, a, b):
    cp1 = (b - a).cross(p1 - a)
    cp2 = (b - a).cross(p2 - a)
    return cp1 * cp2 >= 0

def pointInTriangle(p, a, b, c):
    return sameSide(p, a, b, c) and sameSide(p, b, a, c) and sameSide(p, c, a, b)

#   a---------b
#   |         |
#   d---------c
def pointInRectangle(p, rect):
    return pointInTriangle(p, rect.a, rect.b, rect.c) or pointInTriangle(p, rect.c, rect.d, rect.a)

def rectangleInRectangle(a, b):
    return pointInRectangle(a.a, b) or pointInRectangle(a.b, b) or pointInRectangle(a.c, b) or pointInRectangle(a.d, b) or pointInRectangle(b.a, a) or pointInRectangle(b.b, a) or pointInRectangle(b.c, a) or pointInRectangle(b.d, a)

a = pygame.math.Vector2(2, 0)
b = pygame.math.Vector2(0, 0)
c = pygame.math.Vector2(0, 2)

p = pygame.math.Vector2(1, 1)

pointInTriangle(p, a, b, c)
