import math, sys, pygame
import euclid

def pygame_to_euclid_vector(py_vec):
    return euclid.Vector2(py_vec.x, py_vec.y)

class RigidBody:
    def __init__(self, half_size, mass):
        self.pos = pygame.math.Vector2()
        self.vel = pygame.math.Vector2()
        self.angle = 0
        self.ang_vel = 0

        self.half_size = half_size
        self.mass = mass
        #self.inertia = mass * (half_size.x * half_size.x + half_size.y * half_size.y) * 0.5
        self.inertia = mass * (half_size.x * half_size.x * half_size.y * half_size.y) * (1.0 / 12.0)
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

    def add_world_force(self, world_force, world_offset):
        self.force += world_force
        world_torque = world_offset.cross(world_force)
        print("*************")
        print("force   ="+str(world_force))
        print("offset  ="+str(world_offset))
        print("torque_w="+str(world_torque))
        print("*************")
        self.torque += world_torque

    def add_rel_force(self, force, offset):
        #print("Adding rel force="+str(force)+" @ "+str(offset))
        # Rotate force back to the world coordinate system. We don't want to
        # translate it because its components are positional (they are magnitudes along axes)
        #world_force = self.rot_matrix * force
        # Transform the offset to be world (this rotates and translates it)
        #world_offset = self.relative_to_world(offset)
        #self.add_world_force(world_force, world_offset)
        print("*****************")
        print("force="+str(force))
        force = self.rot_relative_to_world(force)

        self.force += force
        self.torque += self.rot_relative_to_world(offset).cross(force)


        print("rel_force="+str(force))
        print("offset="+str(self.rot_relative_to_world(offset)))
        print("torque="+str(self.rot_relative_to_world(offset).cross(force)))
        print("*****************")

    def update(self, delta_t):
        # Linear integration
        accel = self.force / self.mass
        self.vel += accel * delta_t * 0.9
        self.pos += self.vel * delta_t
        # Reset force
        self.force = pygame.math.Vector2()

        # Angular integration
        ang_accel = self.torque / self.inertia
        self.ang_vel += ang_accel * delta_t * 0.9
        self.angle += self.ang_vel * delta_t
        # Normalize the angle to [0, 2*pi]
        self.angle -= math.trunc(self.angle / (2 * math.pi)) * 2 * math.pi
        # Reset torque
        self.torque = 0

        self.cache_output()

    def cache_output(self):
        # Cache the output transforms
        self.pos_matrix = euclid.Matrix3.new_translate(self.pos.x, self.pos.y)
        self.rot_matrix = euclid.Matrix3.new_rotate(self.angle)
        self.inv_rot_matrix = self.rot_matrix.inverse()

    def rot_relative_to_world(self, relative):
        relative = euclid.Vector2(relative.x, relative.y)
        world = pygame.math.Vector2(self.rot_matrix * relative)
        return world

    def rot_world_to_relative(self, world):
        world = euclid.Vector2(world.x, world.y)
        rel = pygame.math.Vector2(self.inv_rot_matrix * world)
        return rel

    def relative_to_world(self, relative):
        relative = euclid.Vector2(relative.x, relative.y)
        world = pygame.math.Vector2(self.rot_matrix * relative) + self.pos
        return world

    def world_to_relative(self, world):
        world = euclid.Vector2(world.x, world.y)
        rel = pygame.math.Vector2(self.inv_rot_matrix * (world - self.pos))
        return rel

    def rel_point_vel(self, rel):
        tangent = pygame.math.Vector2(-rel.y, rel.x)
        rel_vel = (tangent * self.ang_vel) + self.rot_world_to_relative(self.vel)
        return rel_vel

    def world_point_vel(self, world):
        offset = world - self.pos
        tangent = pygame.math.Vector2(-offset.y, offset.x)
        world_vel = (tangent * self.ang_vel) + self.vel
        return world_vel
