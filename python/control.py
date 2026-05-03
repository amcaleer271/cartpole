import math
import numpy as np
from scipy.linalg import solve_continuous_are

class PID:
    def __init__(self, kp, ki, kd):
        self.error = np.array([0.0,0.0])
        self.error_d = np.array([0.0,0.0])
        self.error_i = np.array([0.0,0.0])

        self.kp = np.array([kp[0], kp[1]])
        self.ki = np.array([ki[0], ki[1]])
        self.kd = np.array([kd[0], kd[1]])

    def __str__(self):
        return "PID"

    def control(self, state, dt=0.001):

        self.error = np.array([state[0], state[2]])
        self.error_d = np.array([state[1], state[3]])
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

    def __str__(self):
        return "LQR"
    
    def control(self, state):
        #state should be a numpy array of form [x, xdot, theta, thetadot]
        u = float(-self.K @ state)
        return u

def create_LQR_mats(m1, m2, L):
    g = 9.81
    A = np.array([[0, 1, 0, 0],[0, 0, (-m2*g)/m1, 0],[0, 0, 0, 1],[0, 0, ((m1+m2)*g)/(m1*L), 0]])
    B = np.array([[0],[1/m1],[0],[-1/(m1*L)]])

    return A, B