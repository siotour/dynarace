import pygame, sys
from car import Wheel

pygame.init()

clock = pygame.time.Clock()

up = False
down = False
elapsed = 0.0
torque = 0.0
brake = False
wheel = Wheel(pygame.math.Vector2(), 10)

def updateKey(key, is_down):
    global up
    global down
    global brake
    if key == pygame.K_UP:
        up = is_down
    elif key == pygame.K_DOWN:
        down = is_down
    elif key == pygame.K_SPACE:
        brake = is_down

def update(delta_t):
    global elapsed
    elapsed += delta_t

    if elapsed >= 0.05:
        global torque
        if up:
            torque += 1
        elif down:
            torque -= 1
        elapsed -= 0.05

    ground_vel = wheel.forward_axis * wheel.vel * wheel.radius
    global brake
    if brake == True:
        print("brake!")
        ground_vel = wheel.forward_axis * 0
    global wheel
    print("****"+str(torque))
    wheel.add_torque(torque)
    wheel.calculate_force(ground_vel, delta_t)


while 1:
    # USER INPUT
    delta_t = clock.tick(30) / 1000.0
    for event in pygame.event.get():
        if not hasattr(event, 'key'): continue
        isDown = event.type == pygame.KEYDOWN
        if event.key == pygame.K_ESCAPE: sys.exit(0)
        else:
            updateKey(event.key, isDown)

    update(delta_t)
    print(str(wheel))
