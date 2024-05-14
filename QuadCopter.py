import numpy as np
from numpy import sin as s, cos as c
from parameters import *
import time

class QuadCopter:
    def __init__(self):
        # Position (m)
        self.x, self.y, self.z = 0.0, 0.0, 0.0

        # Orientation (Euler angles) (radians)
        self.phi, self.theta, self.psi = 0.0, 0.0, 0.0
        np.degrees([self.phi, self.theta, self.psi])

        # Linear velocity (m/s)
        self.vx, self.vy, self.vz = 0.0, 0.0, 0.0

        # Angular velocity (rad/s)
        self.p, self.q, self.r = 0.0, 0.0, 0.0

        # Initial state
        self.linear_position = np.array([self.x, self.y, self.z], dtype=float).reshape(3, 1)
        self.angular_position = np.array([self.phi, self.theta, self.psi], dtype=float).reshape(3, 1)
        self.linear_velocity = np.array([self.vx, self.vy, self.vz], dtype=float).reshape(3, 1)
        self.angular_velocity = np.array([self.p, self.q, self.r], dtype=float).reshape(3, 1)
        self.linear_acceleration = np.array([0, 0, 0], dtype=float).reshape(3, 1)
        self.angular_acceleration = np.array([0, 0, 0], dtype=float).reshape(3, 1)

        # Motor velocities (rad/s)
        self.w1, self.w2, self.w3, self.w4 = 0, 0, 0, 0

        # Inertia (kg*m^2)
        self.I = np.diag([IXX, IYY, IZZ])
        self.I_inverse = np.linalg.inv(self.I)

        # Thrust vector (kg*m/s^2) - Torque vector (kg*m^2/s^2)
        self.update_thrust_torque()

        # Rotation matrix
        self.update_rotation_matrix()

        # Transformation matrix
        self.update_transformation_matrix()

    def __reset__(self):
        self.x, self.y, self.z = 0, 0, 0
        self.phi, self.theta, self.psi = 0, 0, 0
        self.vx, self.vy, self.vz = 0, 0, 0
        self.p, self.q, self.r = 0, 0, 0

    def step(self, velocities, step_time):

        if (time.time() * 1000) % DEBUG_PERIOD_MS < 1:
            self.DEBUG_TIME = True
        else:
            self.DEBUG_TIME = False
    
        self.w1 = velocities[2]
        self.w2 = velocities[0]
        self.w3 = velocities[3]
        self.w4 = velocities[1]

        self.update_thrust_torque()
        self.update_rotation_matrix()
        self.update_transformation_matrix()
        self.update_accelerations()
        self.update_velocities(step_time)
        self.update_position_orientation(step_time)



        return self.angular_velocity, self.linear_acceleration, self.linear_position, self.angular_position, self.linear_velocity
    
    def update_thrust_torque(self):
        self.thrust = np.array(
            [
                0,
                0,
                K * (self.w1**2 + self.w2**2 + self.w3**2 + self.w4**2)
            ],
            dtype=float
        ).reshape(3, 1)

        self.torque = np.array(
            [
                A * L * K * -(self.w2**2 - self.w4**2),
                A * L * K * (self.w1**2 - self.w3**2),
                A * B * -(self.w1**2 - self.w2**2 + self.w3**2 - self.w4**2)
            ],
            dtype=float
        ).reshape(3, 1)

    def update_rotation_matrix(self):
        self.R = np.array(
            [
                c(self.phi) * c(self.psi) - c(self.theta) * s(self.phi) * s(self.psi), -c(self.psi) * s(self.phi) - c(self.phi) * c(self.theta) * s(self.psi), s(self.theta) * s(self.psi),
                c(self.theta) * c(self.psi) * s(self.phi) + c(self.phi) * s(self.psi), c(self.phi) * c(self.theta) + c(self.psi) - s(self.phi) * s(self.psi), -c(self.psi) * s(self.theta),
                s(self.phi) * s(self.theta), c(self.phi) * s(self.theta), c(self.theta)
            ],
            dtype=float
        ).reshape(3, 3)

    def update_transformation_matrix(self):
        self.W = np.array(
            [
                [1, 0, -s(self.theta)],
                [0, c(self.phi), c(self.theta)*s(self.phi)],
                [0, -s(self.phi), c(self.theta)*c(self.phi)]
            ],
            dtype=float
        ).reshape(3, 3)
    
    def update_accelerations(self):
        # Linear acceleration (m/s^2)
        self.linear_acceleration = (self.R @ self.thrust)/MASS_KG
        self.linear_acceleration += np.array([0.0, 0.0, -GRAVITY_M_S2], dtype=float).reshape(3, 1)

        # Angular acceleration (rad/s^2)
        self.angular_acceleration[0] = self.torque[0]/IXX
        self.angular_acceleration[1] = self.torque[1]/IYY
        self.angular_acceleration[2] = self.torque[2]/IZZ
        if DEBUG_ANGULAR_ACCEL and self.DEBUG_TIME:
            # printe(self.angular_acceleration)
            if self.angular_acceleration[0] > 0:
                print("Sağa yatış")
            else:
                print("Sola yatış")

            if self.angular_acceleration[1] > 0:
                print("Pitch up")
            else:
                print("Pitch down")
    
    def update_velocities(self, dt):

        # Tepki kuvveti
        if(self.on_ground()):
            if (self.linear_acceleration[2] <= 0):
                self.linear_acceleration[2] = 0

        # Linear velocity (m/s)
        self.linear_velocity += (self.linear_acceleration * dt)

        # print(self.linear_velocity)

        self.vx, self.vy, self.vz = self.linear_velocity
        # Angular velocity (rad/s) body frame
        # self.angular_velocity += self.angular_acceleration * dt
        phidot = self.angular_acceleration[0] * dt
        thetadot = self.angular_acceleration[1] * dt
        psidot = self.angular_acceleration[2] * dt
        self.angular_velocity[0] = phidot - psidot * s(self.theta)
        self.angular_velocity[1] = c(self.phi) * thetadot + s(self.phi) * psidot * c(self.theta)
        self.angular_velocity[2] = c(self.phi) * psidot * c(self.theta) - s(self.phi) * thetadot

    def update_position_orientation(self, dt):
        # Linear Position (m)
        self.linear_position += self.linear_velocity * dt
        self.x, self.y, self.z = self.linear_position

        # Angular Position (radians) body frame
        self.angular_position += self.angular_velocity * dt
        self.phi, self.theta, self.psi = self.angular_position

    def on_ground(self):
        return self.z <= 0.001
