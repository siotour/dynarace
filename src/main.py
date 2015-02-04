import math, sys, pygame
from car import *
from track import *
from pygame.locals import *
from collision import *
from textbox import *

pygame.init()
screen = pygame.display.set_mode((1024, 768))
clock = pygame.time.Clock()


# CREATE A CAR AND RUN
rect = screen.get_rect()
car = Car('car.png', rect.center)
car_group = pygame.sprite.RenderPlain(car)
segments = list()
segments.append(Rect(100, 100, 100, 100))
segments.append(Rect(240, 100, 100, 100))
track = Track(segments)
text_box = TextBox(Rect(10, 10, 500, 100), 15)
while 1:
    # USER INPUT
    deltat = clock.tick(30) / 1000
    for event in pygame.event.get():
        if not hasattr(event, 'key'): continue
        isDown = event.type == KEYDOWN
        if event.key == K_ESCAPE: sys.exit(0)
        else:
            car.updateKey(event.key, isDown)
    # RENDERING
    screen.fill((0,0,0))
    car_group.update(deltat)
    collisions = pygame.sprite.spritecollide(car, track, False)
    if collisions:
        for segment in collisions:
            if rectangleInRectangle(car.bounding_box, BoundingBox(segment.rect)):
                #print(BoundingBox(segment.rect))
                car.collide()
    car_group.draw(screen)
    text_box.set_text("Car = " + str(car.bounding_box))
    points = [car.bounding_box.a, car.bounding_box.b, car.bounding_box.c, car.bounding_box.d]
    pygame.draw.lines(screen, pygame.Color('red'), True, points)
    #screen.blit(font.render("text", 0, (255, 255, 0)), (10, 10))
    text_box.render(screen)
    points = [(10, 10), (510, 10), (510, 110), (10, 110)]
    pygame.draw.lines(screen, pygame.Color('red'), True, points)
    track.draw(screen)
    pygame.display.flip()
