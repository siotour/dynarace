import math, sys, pygame, pdb, euclid
from collision import BoundingBox
from rigid_body import RigidBody
from rigid_body import pygame_to_euclid_vector


class Car(pygame.sprite.Sprite):
    THROTTLE_TORQUE = 10.0
    BRAKE_TORQUE = 1.0
    MASS = 0.5

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

        self.force_line_pairs = []
        self.vel_line_pairs = []

        # Our rigid body simulator
        half_size = pygame.math.Vector2(self.rect.width * 0.25, self.rect.height * 0.25)
        self.force_offset = pygame.math.Vector2(0, self.rect.height * 0.09)
        self.rigid_body = RigidBody(half_size, self.MASS)

        wheel_pos = []
        wheel_pos.append(half_size)
        wheel_pos.append(pygame.math.Vector2(half_size.x * -1, half_size.y))
        wheel_pos.append(half_size * -1)
        wheel_pos.append(pygame.math.Vector2(half_size.x, half_size.y * -1))

        self.wheels = []
        #self.wheels.append(Wheel(pygame.math.Vector2(0, 0), 0.5))
        self.wheels.append(Wheel(wheel_pos[0], 1))
        self.wheels.append(Wheel(wheel_pos[1], 1))
        self.wheels.append(Wheel(wheel_pos[2], 1))
        self.wheels.append(Wheel(wheel_pos[3], 1))

        self.rigid_body.set_position(pygame.math.Vector2(position[0], position[1]), 3 * math.pi / 2)

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
        #self.wheels[0].add_torque(self.THROTTLE_TORQUE * throttle)
        #self.wheels[1].add_torque(self.THROTTLE_TORQUE * throttle)
        #self.wheels[0].add_torque(self.THROTTLE_TORQUE * throttle)
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
            print("wheel"+str(num))
            num +=1
            rel_ground_vel = self.rigid_body.rel_point_vel(wheel.pos)
            vel_pair = []
            vel_pair.append(self.rigid_body.relative_to_world(wheel.pos))
            vel_pair.append(self.rigid_body.relative_to_world(wheel.pos) + self.rigid_body.rot_relative_to_world(rel_ground_vel))
            self.vel_line_pairs.append(vel_pair)


            #print("rel_vel="+str(rel_ground_vel))
            rel_response_force = wheel.calculate_force(rel_ground_vel, deltat)
            response_force_offset = self.rigid_body.rot_relative_to_world(rel_response_force) + self.rigid_body.relative_to_world(wheel.pos)
            force_pair = []
            force_pair.append(self.rigid_body.relative_to_world(wheel.pos))
            force_pair.append( (response_force_offset.x, response_force_offset.y) )
            self.force_line_pairs.append(force_pair)
            self.rigid_body.add_rel_force(rel_response_force, wheel.pos)
        self.rigid_body.update(deltat)

        #print(self.__str__())
        #print("=====================")

        self.direction = self.rigid_body.angle * 180.0 / math.pi

        self.image = pygame.transform.rotate(self.src_image, -1 * self.direction) # degrees counter-clockwise
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
        # Relative forward and side axes
        self.forward_axis = pygame.math.Vector2()
        self.right_axis = pygame.math.Vector2()
        self.torque = 0.0
        self.vel = 0.0
        self.radius = radius
        self.inertia = self.radius * self.radius
        # Initialize forward and side axes
        self.set_steering_angle(0.0)

    # @param theta angle in radians, counter-clockwise
    def set_steering_angle(self, theta):
        rot_matrix = euclid.Matrix3.new_rotate(theta) # radians counter-clockwise
        forward_axis = euclid.Vector2(0.0, 1.0)
        right_axis = euclid.Vector2(1.0, 0.0)

        self.forward_axis = rot_matrix * forward_axis
        self.right_axis = rot_matrix * right_axis

    def add_torque(self, torque):
        self.torque += torque

    def calculate_force(self, ground_vel, delta_t):
        patch_vel = self.forward_axis * self.vel * self.radius
        differential_vel = patch_vel - ground_vel # positive if car should accelerate; also positive if wheel should decelerate
        #print("diff_vel="+str(differential_vel))

        euclid_differential_vel = pygame_to_euclid_vector(differential_vel)
        euclid_right_axis = pygame_to_euclid_vector(self.right_axis)
        euclid_forward_axis = pygame_to_euclid_vector(self.forward_axis)
        right_vel = euclid_differential_vel.project(euclid_right_axis)
        forward_vel = euclid_differential_vel.project(euclid_forward_axis)
        forward_mag = abs(forward_vel)

        if abs(self.forward_axis + forward_vel) < (abs(self.forward_axis) + forward_mag):
            #print('negated')
            forward_mag *= -1.0

        response_force = right_vel * 5
        response_force += forward_vel * 1.0
        self.torque -= forward_mag * self.radius
        self.vel += self.torque / self.inertia * delta_t * 0.9
        self.torque = 0.0

        print("patch_vel ="+str(patch_vel))
        print("ground_vel="+str(pygame_to_euclid_vector(ground_vel)))
        print("diff_vel  ="+str(differential_vel))
        print("response  ="+str(response_force))
        print("-----------------------------------")

        return response_force

    def __str__(self):
        str_rep = ""
        str_rep += "\ntorque ="+str(self.torque)
        str_rep += "\nvel    ="+str(self.vel)
        return str_rep
