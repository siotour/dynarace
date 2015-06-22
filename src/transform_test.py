import math, sys, pygame, pdb, euclid
from rigid_body import RigidBody
from rigid_body import pygame_to_euclid_vector


half_size = pygame.math.Vector2(10, 15)
rb = RigidBody(half_size, 10)

rb_pos = pygame.math.Vector2(0, 0)
rb_rot = math.pi / 2
rb.set_position(rb_pos, rb_rot)
rb.vel = pygame.math.Vector2(5, 5)
rb.ang_vel = 5.0

rel = pygame.math.Vector2(0, 5)
world = rb.relative_to_world(rel)
ground_vel = rb.point_vel(world)
wheel_vel = rb.world_to_relative(ground_vel)

print("rel     ="+str(rel))
print("world   ="+str(world))
print("grnd_vel="+str(ground_vel))
print("whl_vel ="+str(wheel_vel))
print("---------------------")
