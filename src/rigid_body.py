import math, sys, pygame
import euclid

class RigidBody:
    def __init__(self, half_size, mass):
        self.pos = pygame.math.Vector2()
        self.vel = pygame.math.Vector2()
        self.angle = 0
        self.ang_vel = 0

        self.half_size = half_size
        self.mass = mass
        self.inertia = (1.0 / 12.0) * (half_size.x * half_size.x) * (half_size.y * half_size.y)
        self.rect = (-1 * half_size.x, -1 * half_size.y, 2 * half_size.x, 2 * half_size.y)

        self.pos_matrix = euclid.Matrix3()
        self.rot_matrix = euclid.Matrix3()
        self.inv_rot_matrix = euclid.Matrix3()

        self.force = pygame.math.Vector2()
        self.torque = 0

    def set_position(self, position, angle):
        self.pos = position
        self.angle = angle
        self.cache_output()

    def add_force(self, world_force, world_offset):
        self.force += world_force
        world_torque = world_offset.cross(world_force)
        self.torque += world_torque

    def update(self, delta_t):
        # Linear integration
        accel = self.force * self.mass
        self.vel += accel * delta_t
        self.pos += self.vel * delta_t
        # Reset force
        self.force = pygame.math.Vector2()

        # Angular integration
        ang_accel = self.torque * self.inertia
        self.ang_vel += ang_accel * delta_t
        self.angle += self.ang_vel * delta_t
        # Reset torque
        self.torque = 0

        self.cache_output()

    def cache_output(self):
        # Cache the output transforms
        self.pos_matrix = euclid.Matrix3.new_translate(self.pos.x, self.pos.y)
        self.rot_matrix = euclid.Matrix3.new_rotate(self.angle)
        self.inv_rot_matrix = self.rot_matrix
        self.inv_rot_matrix.inverse

    def relative_to_world(self, relative):
        relative = euclid.Vector2(relative.x, relative.y)
        return pygame.math.Vector2(self.rot_matrix * relative)

    def world_to_relative(self, world):
        world = euclid.Vector2(world.x, world.y)
        return pygame.math.Vector2(self.inv_rot_matrix * world)
