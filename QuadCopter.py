import numpy as np
from numpy import sin as s, cos as c
from parameters import *

class QuadCopter:
    def __init__(self):
        # Position (m)
        self.x, self.y, self.z = 0.0, 0.0, 0.0

        # Orientation (Euler angles) (radians)
        self.phi, self.theta, self.psi = 0.0, 0.0, 0.0

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

    def step(self, velocities):
        self.w1 = velocities[0]
        self.w2 = velocities[1]
        self.w3 = -velocities[2]
        self.w4 = -velocities[3]

        self.update_thrust_torque()
        self.update_rotation_matrix()
        self.update_transformation_matrix()
        self.update_accelerations()
        self.update_velocities()
        self.update_position_orientation()

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
                L * K * (-self.w1**2 + self.w3**2),
                L * K * (-self.w2**2 + self.w4**2),
                B * (-self.w1**2 + self.w2**2 - self.w3**2 + self.w4**2)
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
        self.linear_acceleration += np.array([0.0, 0.0, GRAVITY_M_S2], dtype=float).reshape(3, 1)

        if self.on_ground() and self.linear_acceleration[2] > 0:
            self.linear_acceleration[2] = 0

        # Angular acceleration (rad/s^2)
        self.angular_acceleration = np.dot(self.I_inverse, self.torque)
    
    def update_velocities(self):
        # Linear velocity (m/s)
        self.linear_velocity += (self.linear_acceleration * DT)
        self.vx, self.vy, self.vz = self.linear_velocity

        # Angular velocity (rad/s)
        self.angular_velocity += self.W @ (self.angular_acceleration * DT)
        self.p, self.q, self.r = self.angular_velocity

    def update_position_orientation(self):
        # Linear Position (m)
        self.linear_position += self.linear_velocity * DT
        self.x, self.y, self.z = self.linear_position

        # Angular Position (radians)
        self.angular_position += self.angular_velocity * DT
        self.phi, self.theta, self.psi = self.angular_position

    def on_ground(self):
        return self.z <= 0.001
