import math, sys, pygame, pdb, euclid
from collision import BoundingBox
from rigid_body import RigidBody, pygame_to_euclid_vector
from rigid_body import pygame_to_euclid_vector


class Car(pygame.sprite.Sprite):
    THROTTLE_TORQUE = 10.0
    BRAKE_TORQUE = 15.0
    MASS = 4
    # Should the car stay centered on the screen?
    STAY_CENTERED = False

    def __init__(self, image, screen_pos, world_pos):
        pygame.sprite.Sprite.__init__(self)
        self.world_pos = world_pos
        self.screen_pos = screen_pos
        self.delta_pos = pygame.math.Vector2()
        self.src_image = pygame.image.load(image)
        # Correct for car being upside-down
        self.src_image = pygame.transform.rotate(self.src_image, 180)
        self.rect = self.src_image.get_rect()
        self.rect.center = screen_pos
        self.aabb = self.rect
        self.direction = 3 * math.pi / 2
        self.bounding_box = BoundingBox(self.aabb, self.direction)

        self.forward = self.reverse = self.left = self.right = False

        self.force_line_pairs = []
        self.vel_line_pairs = []

        # Our rigid body simulator
        half_size = pygame.math.Vector2(self.rect.width * 0.25, self.rect.height * 0.25)
        self.rigid_body = RigidBody(2 * half_size, self.MASS)

        wheel_pos = []
        wheel_pos.append(half_size)
        wheel_pos.append(pygame.math.Vector2(half_size.x * -1, half_size.y))
        wheel_pos.append(half_size * -1)
        wheel_pos.append(pygame.math.Vector2(half_size.x, half_size.y * -1))

        self.wheels = []
        self.wheels.append(Wheel(wheel_pos[0], 1))
        self.wheels.append(Wheel(wheel_pos[1], 1))
        self.wheels.append(Wheel(wheel_pos[2], 1))
        self.wheels.append(Wheel(wheel_pos[3], 1))

        self.rigid_body.set_position(self.world_pos, self.direction)

    # angle in degrees counter-clockwise
    def get_angle(self):
        return self.rigid_body.angle# * 180.0 / math.pi

    def get_ang_vel(self):
        return self.rigid_body.ang_vel

    def get_vel(self):
        return self.rigid_body.vel.length()

    # angle in degrees counter-clockwise
    def get_steering(self):
        return self.steering * 180.0 / math.pi

    def get_throttle(self):
        return self.throttle

    def get_brake(self):
        return self.brake

    def __str__(self):
        output =  "throttle=" + str(self.get_throttle()) + "\n"
        output += "brake   =" + str(self.get_brake()) + "\n"
        output += "steering=" + "{:3.1f}".format(self.get_steering()) + "\n"
        output += "velocity=" + "{:3.1f}".format(self.get_vel()) + "\n"
        output += "angle   =" + "{:3.1f}".format(self.get_angle()) + "\n"
        output += "ang vel =" + "{:3.1f}".format(self.get_ang_vel()) + "\n"
        for wheel in self.wheels:
            output += "wheel=" + str(wheel) + "\n"
        return output

    # Returns an array of points which can be used to draw lines representing the car's wheels
    def get_wheel_lines(self):
        line_length = 20.0
        wheel_vec_pairs = []

        # Rotate front wheels with steering angle
        rot_matrix = euclid.Matrix3.new_rotate(self.steering)
        for wheel in self.wheels[:2]:
            pos = wheel.pos
            vec = euclid.Vector2(0, -line_length / 2)
            wheel_vec_pair = []
            wheel_vec_pair.append(pygame.math.Vector2(rot_matrix * vec) + pos)
            vec.y = line_length / 2
            wheel_vec_pair.append(pygame.math.Vector2(rot_matrix * vec) + pos)
            wheel_vec_pairs.append(wheel_vec_pair)

        for wheel in self.wheels[2:]:
            pos = wheel.pos
            wheel_vec_pair = []
            wheel_vec_pair.append(pygame.math.Vector2(pos.x, pos.y - line_length / 2))
            wheel_vec_pair.append(pygame.math.Vector2(pos.x, pos.y + line_length / 2))
            wheel_vec_pairs.append(wheel_vec_pair)

        wheel_point_pairs = []

        for pair in wheel_vec_pairs:
            point_pair = []
            for vec in pair:
                vec_rot = self.rigid_body.relative_to_world(vec)
                point_pair.append( (vec_rot.x, vec_rot.y) )
            wheel_point_pairs.append(point_pair)
            #print("("+str(vec.x)+","+str(vec.y)+" => ("+str(vec_rot.x)+","+str(vec_rot.y)+")")
        return wheel_point_pairs

    def get_force_lines(self):
        force_lines = self.force_line_pairs
        print("num pairs="+str(len(self.force_line_pairs)))
        self.force_line_pairs = []
        return force_lines

    def get_vel_lines(self):
        vel_lines = self.vel_line_pairs
        self.vel_line_pairs = []
        return vel_lines

    def set_steering(self, theta):
        self.wheels[0].set_steering_angle(theta)
        self.wheels[1].set_steering_angle(theta)

    def set_throttle(self, throttle):
        self.wheels[1].add_torque(self.THROTTLE_TORQUE * throttle)
        self.wheels[0].add_torque(self.THROTTLE_TORQUE * throttle)
        self.wheels[2].add_torque(self.THROTTLE_TORQUE * throttle)
        self.wheels[3].add_torque(self.THROTTLE_TORQUE * throttle)

    def set_brake(self, brake):
        for wheel in self.wheels:
            torque = self.BRAKE_TORQUE * -wheel.vel * brake
            wheel.add_torque(torque)

    def updateKey(self, key, pressed):
        if key == pygame.K_UP:
            self.forward = pressed
        elif key == pygame.K_DOWN:
            self.reverse = pressed
        elif key == pygame.K_LEFT:
            self.right = pressed
        elif key == pygame.K_RIGHT:
            self.left = pressed


    def processKeys(self):
        if self.forward:
            self.throttle = 100
        else:
            self.throttle = 0

        if self.reverse:
            self.brake = 1
        else:
            self.brake = 0

        self.set_brake(self.brake)
        self.set_throttle(self.throttle)

        if self.left:
            self.steering = math.pi / 4
        elif self.right:
            self.steering = -math.pi / 4
        else:
            self.steering = 0

        self.set_steering(self.steering)

    def update(self, deltat):
        self.processKeys()
        num = 0
        for wheel in self.wheels:
            rel_ground_vel = self.rigid_body.rel_point_vel(wheel.pos)
            vel_pair = []
            vel_pair.append(self.rigid_body.relative_to_world(wheel.pos))
            vel_pair.append(self.rigid_body.relative_to_world(wheel.pos) + self.rigid_body.rot_relative_to_world(rel_ground_vel))
            self.vel_line_pairs.append(vel_pair)

            rel_response_force = wheel.calculate_force(rel_ground_vel, deltat)
            response_force_offset = self.rigid_body.rot_relative_to_world(rel_response_force) + self.rigid_body.relative_to_world(wheel.pos)
            force_pair = []
            force_pair.append(self.rigid_body.relative_to_world(wheel.pos))
            force_pair.append( (response_force_offset.x, response_force_offset.y) )
            self.force_line_pairs.append(force_pair)
            self.rigid_body.add_rel_force(rel_response_force, wheel.pos)

        self.rigid_body.update(deltat)
        self.direction = self.rigid_body.angle
        self.delta_pos += self.rigid_body.pos - self.world_pos
        self.world_pos = self.rigid_body.pos

        self.image = pygame.transform.rotate(self.src_image, -1 * self.direction * 180 / math.pi) # degrees counter-clockwise
        self.rect = self.image.get_rect()
        self.rect.center = self.screen_pos if self.STAY_CENTERED else self.rigid_body.pos
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
    # @param position [pygame.math.Vector2] center position of wheel relative to a rigid body
    # @param radius [float] radius of the wheel. Used to calculate inertia
    # @todo Add a mass parameter which will factor into the wheel's inertia
    def __init__(self, position, radius):
        self.pos = position
        self.torque = 0.0
        self.vel = 0.0
        self.radius = radius
        self.inertia = self.radius * self.radius
        # Initialize forward and side axes
        self.set_steering_angle(0.0)

    # @param theta [float] angle in radians, counter-clockwise
    def set_steering_angle(self, theta):
        rot_matrix = euclid.Matrix3.new_rotate(theta) # radians counter-clockwise
        forward_axis = euclid.Vector2(0.0, 1.0)
        right_axis = euclid.Vector2(1.0, 0.0)

        self.forward_axis = rot_matrix * forward_axis
        self.right_axis = rot_matrix * right_axis

    # @param torque [float] forward torque to apply to the wheel
    def add_torque(self, torque):
        self.torque += torque

    # Calculates the force generated by the wheel on the rigid body it's attached to and
    # updates the wheel's physics model relative to an amount of elapsed time.
    # @param ground_vel [pygame.math.Vector2] relative velocity of the ground
    # @param delta_t [float] time in seconds since the last call to calculate_force()
    # @return [pygame.math.Vector2] force generated by the wheel on the rigid body it's attached to
    def calculate_force(self, ground_vel, delta_t):
        # Velocity of the tire patch. Positive is the tirepatch moving backwards in favor
        # of forward movement.
        patch_vel = self.forward_axis * self.vel * self.radius
        # The difference between the ground velocity and the tire patch velocity. Positive
        # if the car should accelerate and the wheel should decelerate
        diff_vel = patch_vel - ground_vel

        # Use euclid vectors to calculate some vector projections
        euclid_diff_vel = pygame_to_euclid_vector(diff_vel)
        euclid_right_axis = pygame_to_euclid_vector(self.right_axis)
        euclid_forward_axis = pygame_to_euclid_vector(self.forward_axis)

        # Differential velocity along right axis
        right_diff_vel = euclid_diff_vel.project(euclid_right_axis)
        # Differential velocity component along forward axis
        forward_diff_vel = euclid_diff_vel.project(euclid_forward_axis)
        # Signed magnitude of the forward differential velocity component
        forward_mag = abs(forward_diff_vel)
        if abs(self.forward_axis + forward_diff_vel) < (abs(self.forward_axis) + forward_mag):
            forward_mag *= -1.0

        # Response force perpendicular to the wheel's direction of rotation. Large friction coefficient because it's
        # hard to push a wheel sideways
        response_force = right_diff_vel * 2
        # Response force parallel to the wheel's direction of rotation. Small friction coefficient because it's
        # easy to rotate a wheel
        response_force += forward_diff_vel
        # Force of the ground will add a torque to the wheel. This and the torque previously added to the wheel
        # will change its velocity
        self.torque -= forward_mag * self.radius
        self.vel += self.torque / self.inertia * delta_t * 0.9
        self.torque = 0.0
        # Return the complete response force vector
        return response_force

    def __str__(self):
        str_rep = ""
        str_rep += "\ntorque ="+str(self.torque)
        str_rep += "\nvel    ="+str(self.vel)
        return str_rep
