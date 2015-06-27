import math, sys, pygame, euclid

# Converts a pygame vector to a euclid vector
# @param py_vec [pygame.math.Vector2] pygame vector
# @return [euclid.Vector2] vector converted to a euclid vector
def pygame_to_euclid_vector(py_vec):
    return euclid.Vector2(py_vec.x, py_vec.y)

class RigidBody:
    # @note mass is assumed to be uniformly distributed throughout the rigid body's rectangular area
    # @param size [pygame.math.Vector2] rectangular dimensions of the rigid body
    # @param mass [float] mass of the rigid body
    def __init__(self, size, mass):
        self.pos = pygame.math.Vector2()
        self.vel = pygame.math.Vector2()
        self.angle = 0
        self.ang_vel = 0

        self.mass = mass
        # Polar mass moment of inertia for a rectangular plate
        self.inertia = (mass * ((size.x * size.x) + (size.y * size.y))) / 12.0

        self.force = pygame.math.Vector2()
        self.torque = 0

        self.cache_matrices()

    # Set the position and rotation of the rigid body
    # @param position [pygame.math.Vector2] center of rigid body in world space
    # @param angle [float] rotation in radians counter-clockwise
    def set_position(self, position, angle):
        self.pos = position
        self.angle = angle
        self.cache_matrices()

    # @param force [pygame.math.Vector2] force in world space
    # @param pos [pygame.math.Vector2] position where force is applied in world space
    def add_world_force(self, force, pos):
        self.force += force
        # Offset relative to rigid body center in world axes
        offset = position - self.pos
        self.torque += offset.cross(force)

    # @param force [pygame.math.Vector2] force in relative space
    # @param offset [pygame.math.Vector2] offset where force is applied in relative space
    def add_rel_force(self, force, offset):
        world_force = self.rot_relative_to_world(force)
        self.force += world_force
        self.torque += offset.cross(force)

    # Performs integration over time period delta_t with the
    # forces last added to the rigid body.
    # @param delta_t [float] time elapsed in seconds since last update
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

        self.cache_matrices()

    # Rotate a point from the relative coordinate system to the world coordinate system
    # @param relative [pygame.math.Vector2] point in relative coordinate system
    # @return [pygame.math.Vector2] point rotated into world coordinate system
    def rot_relative_to_world(self, relative):
        relative = euclid.Vector2(relative.x, relative.y)
        world = pygame.math.Vector2(self.rot_matrix * relative)
        return world

    # Rotate a point from world to coordinate system.
    # @param world [pygame.math.Vector2] point in world coordinate system
    # @return [pygame.math.Vector2] point rotated into relative coordinate system
    def rot_world_to_relative(self, world):
        world = euclid.Vector2(world.x, world.y)
        rel = pygame.math.Vector2(self.inv_rot_matrix * world)
        return rel

    # Rotate and translate a point from relative to world space.
    # @param relative [pygame.math.Vector2] point in relative space
    # @return [pygame.math.Vector2] point rotated and translated into world space
    def relative_to_world(self, relative):
        relative = euclid.Vector2(relative.x, relative.y)
        world = pygame.math.Vector2(self.rot_matrix * relative) + self.pos
        return world

    # Rotate and translate a point from world to relative space.
    # @param world [pygame.math.Vector2] point in world space
    # @param [pygame.math.Vector2] point rotated and translated into relative space
    def world_to_relative(self, world):
        world = euclid.Vector2(world.x, world.y)
        rel = pygame.math.Vector2(self.inv_rot_matrix * (world - self.pos))
        return rel

    # Calcualtes the velocity of a point attached to the rigid body in relative space
    # @param rel [pygame.math.Vector2] point in relative space
    # @return [pygame.math.Vector2] velocity of the point in relative space if it were attached to the rigid body
    def rel_point_vel(self, rel):
        tangent = pygame.math.Vector2(-rel.y, rel.x)
        rel_vel = (tangent * self.ang_vel) + self.rot_world_to_relative(self.vel)
        return rel_vel

    # Calculates the velocity of a pont attached to the rigid body in world space
    # @param world [pygame.math.Vector2] point in world space
    # @return [pygame.math.Vector2] velocity of the point in world space if it were attached to the rigid body
    def world_point_vel(self, world):
        offset = world - self.pos
        tangent = pygame.math.Vector2(-offset.y, offset.x)
        world_vel = (tangent * self.ang_vel) + self.vel
        return world_vel

    # Caches the rotation and inverse-rotation matrices for the rigid body
    def cache_matrices(self):
        # Cache rotation matrices
        self.rot_matrix = euclid.Matrix3.new_rotate(self.angle)
        self.inv_rot_matrix = self.rot_matrix.inverse()
