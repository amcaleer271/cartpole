import numpy as np
from control import create_LQR_mats

class KalmanFilter():
    def __init__(self, A, B, dt):
        self.x_hat = np.array([0.0,0.0,0.0,0.0])
        self.P = np.diag([1,1,1,1])
        self.Ad = np.eye(4) + A*dt
        self.Bd = B * dt

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ])

        self.R = np.array([
            [ 0.25**2, 0.0 ],
            [ 0.0, 0.25**2 ]
        ])

        self.Q = np.diag([1e-5, 1e-3, 1e-5, 1e-3])


    def estimate(self, u, z):
        #prediction step
        

        self.x_hat = self.Ad @ self.x_hat + (self.Bd @ np.array([u])).flatten()
        self.P = self.Ad@self.P@(np.transpose(self.Ad)) + self.Q

        #measurement step
        self.z = z
        self.y = self.z - self.H@self.x_hat   #compute innovation

        self.S = self.H@self.P@np.transpose(self.H) + self.R
        self.K = self.P@np.transpose(self.H)@np.linalg.inv(self.S)

        #correction step
        self.x_hat = self.x_hat + self.K@self.y
        self.P = (np.eye(4)-self.K@self.H)@self.P

        return self.x_hat