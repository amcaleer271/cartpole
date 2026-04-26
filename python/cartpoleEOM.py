import math
import numpy as np
import matplotlib.pyplot as plt

g = 9.81 #m/s2
class Cartpole:

    def __init__(self, m1, m2, L):

        self.use_control = False

        self.m1 = m1
        self.m2 = m2
        self.L = L
        self.theta = 0.01
        self.theta_d = 0
        self.theta_dd = 0
        self.x = 0
        self.xd = 0
        self.xdd = 0

        self.acc = np.array([self.xdd, self.theta_dd])
        self.vel = np.array([self.xd, self.theta_d])
        self.pos = np.array([self.x, self.theta])

        self.t_data = []
        self.x_data = []
        self.theta_data = []

    def update(self, u,t):
        #update the state of the cartpole system after a short time t
        
        #EOM:

        c = math.cos(self.theta)
        s = math.sin(self.theta)

        self.M_mat = np.array([
            [self.m1 + self.m2, self.m2 * self.L * c],
            [c, self.L]
        ])

        self.F = np.array([
            u + self.m2 * self.L * (self.theta_d**2) * s,
            g * s   
        ])


        self.acc = np.linalg.solve(self.M_mat, self.F)

        #Update using EOM
        self.vel = self.vel + self.acc * t
        self.pos = self.pos + self.vel * t

        self.x, self.theta = self.pos
        self.xd, self.theta_d = self.vel
        self.xdd, self.theta_dd = self.acc

        return 0
    
    def plot_results(self):
        plt.figure()

        plt.subplot(2,1,1)
        plt.plot(self.t_data, self.x_data)
        plt.ylabel("Cart Position x (m)")
        plt.grid()

        plt.subplot(2,1,2)
        plt.plot(self.t_data, self.theta_data)
        plt.ylabel("Pole Angle θ (rad)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.tight_layout()
        plt.show()

        return 0
        
    def simulate(self, dt, steps):
        for i in range(steps):
            t = i * dt
            self.update(0.0, dt)

            self.t_data.append(t)
            self.x_data.append(self.x)
            self.theta_data.append(self.theta)
        
        return 0
    

cartpole = Cartpole(1,1,1)
cartpole.simulate(0.001, 50000)
cartpole.plot_results()


