import math
import numpy as np
from scipy.linalg import solve_continuous_are

def bang_bang(state, deadzone, force):
    u = 0.0
    if state[1] > deadzone:
        u = 1 * force
    elif state[1] < deadzone:
        u = -1.0 * force
    else:
        u = 0.0
    return u

class PID:
    def __init__(self, kp, ki, kd):
        self.error = np.array([0.0,0.0])
        self.error_d = np.array([0.0,0.0])
        self.error_i = np.array([0.0,0.0])

        self.kp = np.array([kp[0], kp[1]])
        self.ki = np.array([ki[0], ki[1]])
        self.kd = np.array([kd[0], kd[1]])

    def control(self, state, dt):

        self.error = np.array([state[0], state[1]])
        self.error_d = np.array([state[2], state[3]])
        self.error_i = self.error_i + self.error * dt

        u = self.kp @ np.transpose(self.error) + self.ki @ np.transpose(self.error_i) + self.kd @ np.transpose(self.error_d)

        return u
    
class LQR:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        #Solve the Riccati equation
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)

        #Compute gains
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

    def control(self, state):
        #state should be a numpy array of form [x, xdot, theta, thetadot]
        u = float(-self.K @ state)
        return u
