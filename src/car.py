import math, sys, pygame, pdb
from collision import BoundingBox
from rigid_body import RigidBody

def pygame_to_euclid_vector(py_vec):
    return euclid.Vector2(py_vec.x, py_vec.y)

class Car(pygame.sprite.Sprite):
    ACCEL_FORCE=pygame.math.Vector2(0, -10)
    DECEL_FORCE = ACCEL_FORCE * -1
    RIGHT_FORCE = pygame.math.Vector2(-0.01, 0)
    LEFT_FORCE = RIGHT_FORCE * -1
    MASS = 100

    def __init__(self, image, position):
        pygame.sprite.Sprite.__init__(self)
        self.src_image = pygame.image.load(image)
        # Correct for car being upside-down
        #self.src_image = pygame.transform.rotate(self.src_image, 180)
        self.rect = self.src_image.get_rect()
        self.position = self.old_position = position
        self.speed = self.old_speed = 0
        self.direction = self.old_direction = 0
        self.forward = self.reverse = self.left = self.right = False
        self.aabb = self.rect
        self.bounding_box = BoundingBox(self.aabb, 0)


        # Our rigid body simulator
        half_size = pygame.math.Vector2(self.rect.width * 0.05, self.rect.height * 0.05)
        self.force_offset = pygame.math.Vector2(0, self.rect.height * 0.09)
        self.rigid_body = RigidBody(half_size, self.MASS)


    def updateKey(self, key, pressed):
        if key == pygame.K_UP:
            self.forward = pressed
        elif key == pygame.K_DOWN:
            self.reverse = pressed
        elif key == pygame.K_LEFT:
            self.left = pressed
        elif key == pygame.K_RIGHT:
            self.right = pressed


    def processKeys(self):
        if self.forward:
            #pdb.set_trace()
            relative_force = self.rigid_body.world_to_relative(self.ACCEL_FORCE)
            force_offset = self.rigid_body.world_to_relative(self.force_offset)
            self.rigid_body.add_force(relative_force, force_offset)
        elif self.reverse:
            relative_force = self.rigid_body.world_to_relative(self.DECEL_FORCE)
            force_offset = self.rigid_body.world_to_relative(self.force_offset)
            self.rigid_body.add_force(relative_force, force_offset)

        if self.left:
            #pdb.set_trace()
            relative_force = self.rigid_body.world_to_relative(self.LEFT_FORCE)
            force_offset = self.rigid_body.world_to_relative(self.force_offset)
            self.rigid_body.add_force(relative_force, force_offset)
        elif self.right:
            relative_force = self.rigid_body.world_to_relative(self.RIGHT_FORCE)
            force_offset = self.rigid_body.world_to_relative(self.force_offset)
            self.rigid_body.add_force(relative_force, force_offset)

    def update(self, deltat):
        if deltat >= 0:
            self.processKeys()
            self.rigid_body.update(deltat)

        self.direction = self.rigid_body.angle * 180.0 / math.pi

        self.image = pygame.transform.rotate(self.src_image, -1 * self.direction)
        self.rect = self.image.get_rect()
        self.rect.center = self.rigid_body.pos
        self.aabb.center = self.rigid_body.pos
        self.bounding_box = BoundingBox(self.aabb, self.direction)

    def collide(self):
        self.speed = 0
        self.position = self.old_position
        self.direction = self.old_direction
        self.image = pygame.transform.rotate(self.src_image, self.direction)
        self.rect = self.image.get_rect()
        self.rect.center = self.position
        self.aabb.center = self.position
        self.bounding_box = BoundingBox(self.aabb, -1 * self.direction)



class Wheel:

    def __init__(self, position, radius):
        self.pos = position
        self.forward_axis = pygame.math.Vector2()
        self.side_axis = pygame.math.Vector2()
        self.torque = 0.0
        self.vel = 0.0
        self.radius = radius
        self.inertia = self.radius * self.radius

        set_steering_angle(0.0)

    def set_steering_angle(self, theta):
        rot_matrix = euclid.Matrix3.new_rotate(theta)
        forward_axis = euclid.Vector2(0.0, 1.0)
        side_axis = euclid.Vector2(-1.0, 0.0)

        self.forward_axis = rot_matrix * forward_axis
        self.side_axis = rot_matrix * side_axis

    def add_torque(self, torque):
        self.torque += torque

    def calculate_force(ground_vel, delta_t):
        patch_vel = self.forward_axis * self.vel * self.radius
        differential_vel = ground_vel - patch_vel

        euclid_ground_vel = pygame_to_euclid_vector(ground_vel)
        euclid_side_axis = pygame_to_euclid_vector(self.side_axis)
        euclid_forward_axis = pygame_to_euclid_vector(self.forward_axis)
        side_vel = euclid_ground_vel.project(euclid_side_axis)
        forward_vel = euclid_ground_vel.project(euclid_forward_axis)
        forward_mag = abs(forward_vel)

        response_force = side_vel * -2.0
        response_force -= forward_vel

        self.torque += forward_mag * self.radius
        self.vel += self.torque / self.inertia * delta_t
        self.torque = 0.0

        return response_force
