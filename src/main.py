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
car = Car('car.png', pygame.math.Vector2(rect.center), pygame.math.Vector2())
car_group = pygame.sprite.RenderPlain(car)
segments = list()
#segments.append(Rect(100, 100, 100, 100))
#segments.append(Rect(240, 100, 100, 100))
track = Track(segments)
text_box = TextBox(Rect(10, 10, 500, 500), 15)
while 1:
    # USER INPUT
    deltat = clock.tick(30) / 1000.0
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

    text = ""
    text += "pos=" + str(car.rigid_body.pos) + "\n"
    text += "angle=" + str(car.rigid_body.angle) + "\n"

    text_box.set_text(car.__str__())
    #points = [car.bounding_box.a, car.bounding_box.b, car.bounding_box.c, car.bounding_box.d]
    #pygame.draw.lines(screen, pygame.Color('red'), True, points)

    wheel_point_pairs = car.get_wheel_lines()
    for pair in wheel_point_pairs:
        pygame.draw.line(screen, pygame.Color('red'), pair[0], pair[1])

    force_point_pairs = car.get_force_lines()
    for pair in force_point_pairs:
        pygame.draw.line(screen, pygame.Color('blue'), pair[0], pair[1])

    vel_point_pairs = car.get_vel_lines()
    for pair in vel_point_pairs:
        pygame.draw.line(screen, pygame.Color('green'), pair[0], pair[1])

    pos = car.rigid_body.pos
    pygame.draw.circle(screen, pygame.Color('red'), (int(pos.x), int(pos.y)), 3)
    #screen.blit(font.render("text", 0, (255, 255, 0)), (10, 10))
    text_box.render(screen)
    #points = [(10, 10), (510, 10), (510, 510), (10, 510)]
    #pygame.draw.lines(screen, pygame.Color('red'), True, points)
    track.draw(screen)
    pygame.display.flip()
