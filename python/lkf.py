import numpy as np
from control import create_LQR_mats

class LinearKalmanFilter():
    def __init__(self, A, B, dt):

        self.x_hat = np.array([0.0,0.0,0.0,0.0])  #Initial estimate of state [x, xd, theta, thetad]
        self.P = np.diag([1,1,1,1]) #Initial Covariance matrix

        self.Ad = np.eye(4) + A*dt   #Discrete time dynamics
        self.Bd = B * dt

        #Measurement model H , dictates which elements of the state can be obtained from sensors
        self.H = np.array([         
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0]
        ])

        #Measurement noise covariance R, comes from stddev ^2
        self.R = np.array([
            [ 0.25**2, 0.0 ],
            [ 0.0, 0.25**2 ]
        ])

        #Process noise covariance Q, models uncertainty in dynamics
        self.Q = np.diag([1e-5, 1e-3, 1e-5, 1e-3])


    def estimate(self, u, z) -> np.array:
        """
        The estimate function returns cartpole the estimated state from the kalman filter
        
        :param u: current control input
        :param z: current measurement of the system in form [x, theta] (2x1 np array)

        :updates: 
        self.P, self.K

        :returns: np.array([x, x_dot, theta, theta_dot])
        """
        #prediction step using discrete time model
        self.x_hat = self.Ad @ self.x_hat + (self.Bd @ np.array([u])).flatten()
        
        #Adjust covariance 
        self.P = self.Ad@self.P@(np.transpose(self.Ad)) + self.Q

        #measurement step
        self.z = z
        self.y = self.z - self.H@self.x_hat   #compute innovation

        self.S = self.H@self.P@np.transpose(self.H) + self.R  #innovation covariance
        self.K = self.P@np.transpose(self.H)@np.linalg.inv(self.S)   #kalman gain

        #correction step
        self.x_hat = self.x_hat + self.K@self.y  #correct previous estimate using measurement error
        self.P = (np.eye(4)-self.K@self.H)@self.P    #update covariance

        return self.x_hat