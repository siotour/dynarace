import math, sys, pygame
from pygame.locals import *
from collision import BoundingBox

class Car(pygame.sprite.Sprite):
    TOP_SPEED = 20
    TOP_SPEED_REVERSE = TOP_SPEED * -0.6
    ACCEL = 1.5
    TURN_SPEED = 6
    def __init__(self, image, position):
        pygame.sprite.Sprite.__init__(self)
        self.src_image = pygame.image.load(image)
        # Correct for car being upside-down
        self.src_image = pygame.transform.rotate(self.src_image, 180)
        self.rect = self.src_image.get_rect()
        self.position = self.old_position = position
        self.speed = self.old_speed = 0
        self.direction = self.old_direction = 0
        self.forward = self.reverse = self.left = self.right = False
        self.aabb = self.rect
        self.bounding_box = BoundingBox(self.aabb, 0)


    def updateKey(self, key, pressed):
        if key == K_UP:
            self.forward = pressed
        elif key == K_DOWN:
            self.reverse = pressed
        elif key == K_LEFT:
            self.left = pressed
        elif key == K_RIGHT:
            self.right = pressed


    def processKeys(self):
        if self.forward:
            self.speed = self.TOP_SPEED if self.speed + self.ACCEL >= self.TOP_SPEED else self.speed + self.ACCEL
        elif self.reverse:
            self.speed = self.TOP_SPEED_REVERSE if self.speed - self.ACCEL <= self.TOP_SPEED_REVERSE else self.speed - self.ACCEL

        self.old_direction = self.direction

        if self.left:
            self.direction = (self.direction + self.TURN_SPEED) % 360
        elif self.right:
            self.direction = (self.direction - self.TURN_SPEED) % 360

    def update(self, deltat):
        if deltat >= 0:
            self.processKeys()
        x, y = self.old_position = self.position
        x += self.speed*math.sin(self.direction * math.pi / 180)
        y += self.speed*math.cos(self.direction * math.pi / 180)
        self.position = (x, y)
        self.image = pygame.transform.rotate(self.src_image, self.direction)
        self.rect = self.image.get_rect()
        self.rect.center = self.position
        self.aabb.center = self.position
        self.bounding_box = BoundingBox(self.aabb, -1 * self.direction)

    def collide(self):
        self.speed = 0
        self.position = self.old_position
        self.direction = self.old_direction
        self.image = pygame.transform.rotate(self.src_image, self.direction)
        self.rect = self.image.get_rect()
        self.rect.center = self.position
        self.aabb.center = self.position
        self.bounding_box = BoundingBox(self.aabb, -1 * self.direction)
