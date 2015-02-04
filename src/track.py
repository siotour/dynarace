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
    def __init__(self, segments):
        pygame.sprite.Group.__init__(self)
        image = pygame.image.load("track_segment.png")
        for rect in segments:
            self.add(TrackSegment(image, rect))
