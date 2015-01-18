import math, sys, array, pygame
from pygame.locals import *

class TrackSegment(pygame.sprite.DirtySprite):
    def __init__(self, image, rect):
        pygame.sprite.Sprite.__init__(self)
        self.image = image
        self.rect = rect
        self.dirty = 2
        print(self.image)

class Track(pygame.sprite.Group):
    def __init__(self):
        pygame.sprite.Group.__init__(self)
        image = pygame.image.load("track_segment.png")
        self.segments = array.array
        rects = [
            Rect(100, 100, 100, 100)
        ]
        for rect in rects:
            self.add(TrackSegment(image, rect))
