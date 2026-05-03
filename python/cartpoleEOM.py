"""
A simulation to test different contoller for the cartpole controls problem.
The objective is to balance a pendulum upright by applying a horizontal force to the cart is is mounted to
The cart is able to move +/- 1 m from its initial position, and should actively try to center in the arena
A visualizer can be used by setting the parameter in init to True
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from control import PID, LQR
import random
from visualization import *

g = 9.81 #m/s2

class Cartpole:

    def __init__(self, m1, m2, L, controller, use_vis=False, theta0=random.uniform(-0.5, 0.5), noisy=False):

        #PARAMETERS
        self.controller = controller
        self.use_visualization = use_vis
        self.noisy = noisy

        #fetch system parameters
        self.m1 = m1
        self.m2 = m2
        self.L = L

        #initialize angle to random value in range
        self.theta = theta0
        self.theta0 = self.theta
        print(f"Starting angle is {self.theta} rads, {self.theta * 180.0 / math.pi} degs")

        #initialize all other state variables to 0
        self.theta_d = 0.0
        self.theta_dd = 0.0
        self.x = 0.0
        self.xd = 0.0
        self.xdd = 0.0

        self.x_noisy = 0.0
        self.xd_noisy = 0.0
        self.theta_noisy = 0.0
        self.thetad_noisy = 0.0

        self.u = 0.0

        #create np arrays for each pose element
        self.acc = np.array([self.xdd, self.theta_dd])
        self.vel = np.array([self.xd, self.theta_d])
        self.pos = np.array([self.x, self.theta])

        #create empty arrays for plotting
        self.t_data = []
        self.x_data = []
        self.xd_data = []
        self.u_data = []
        self.theta_data = []
        
        if self.noisy:
            self.x_noisy_data = []
            self.xd_noisy_data = []
            self.theta_noisy_data = []
            self.thetad_noisy_data = []

    def update(self, u, t):
        #update the state of the cartpole system after a short timestep t
        c = math.cos(self.theta)
        s = math.sin(self.theta)

        #EOM of form M_mat * [xdd, theta_dd] = F
        self.M_mat = np.array([
            [self.m1 + self.m2, self.m2 * self.L * c],
            [self.m2 * c, self.m2 * self.L]
        ])

        self.F = np.array([
            u + self.m2 * self.L * (self.theta_d**2) * s,
            self.m2 * g * s   
        ])

        #Solve for the current linear and angular acceleration
        self.acc = np.linalg.solve(self.M_mat, self.F) 

        #Update using EOM
        self.vel = self.vel + self.acc * t
        self.pos = self.pos + self.vel * t

        self.x, self.theta = self.pos
        self.xd, self.theta_d = self.vel
        self.xdd, self.theta_dd = self.acc

        if self.noisy:
            self.x_noisy = self.x + random.gauss(0.0, 0.50)
            self.theta_noisy = self.theta + random.gauss(0.0, 0.50)
            self.xd_noisy = self.xd + random.gauss(0.0, 0.50)
            self.thetad_noisy = self.theta_d + random.gauss(0.0, 0.50)
    
    #Plot pose using matplotlib
    def plot_results(self):
        plt.figure()
        plt.suptitle(f"{self.controller}")

        plt.subplot(5,2,1)
        plt.plot(self.t_data, self.x_data)
        plt.ylabel("Cart Position x (m)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.subplot(5,2,3)
        plt.plot(self.t_data, self.theta_data)
        plt.ylabel("Pole Angle θ (deg)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.subplot(5,2,5)
        plt.plot(self.t_data, self.u_data)
        plt.ylabel("Control Input u (N)")
        plt.xlabel("Time (s)")
        plt.grid()

        if self.noisy:
            plt.subplot(5,2,2)
            plt.plot(self.t_data, self.x_noisy_data)
            plt.ylabel("Measured Cart Position x (m)")
            plt.xlabel("Time (s)")
            plt.grid()

            plt.subplot(5,2,4)
            plt.plot(self.t_data, self.theta_noisy_data)
            plt.ylabel("Measured Pole Angle θ (deg)")
            plt.xlabel("Time (s)")
            plt.grid()

        plt.tight_layout()
        plt.show()

        
    def create_metrics(self):
    
        print("=========================================")
        print(f"{self.controller} Metrics")
        print("=========================================")
        print("----------Setup----------")
        print(f"Mass 1 = {self.m1}")
        print(f"Mass 2 = {self.m2}")
        print(f"Length = {self.L}")
        #Settling time - time until response remains within 5% of initial value
        print("----------Settling----------")
        print(f"Target (5% of intial value): {0.05 * self.theta0}")
        print(f"Final value: {self.theta_data[-1]}")

        if abs(self.theta_data[-1]) > abs(0.05 * self.theta0):
            print(f"System did not reach < 5% settling")
        else:
            self.rev_theta = self.theta_data[::-1]
            self.settled_index = len(self.rev_theta) - next((i for i, x in enumerate(self.rev_theta) if abs(x) > abs(0.05 * self.theta0)), None)

            print(f"System settled after {self.t_data[self.settled_index]} s")

        
        print("----------Control----------")

        #RMS control input
        self.u_RMS = 0.0
        self.work = 0.0
        for i, u in enumerate(self.u_data):
            self.u_RMS += u ** 2
            self.work += abs(u * self.xd_data[i]) * self.dt
        self.u_RMS = math.sqrt(self.u_RMS / (i+1))
        
        print(f"RMS Control input: {self.u_RMS} N")
        print(f"Maximum Control Input {max(self.u_data)}")        
        print(f"Work done: {self.work} J")    

    def simulate(self, dt, steps):
        self.dt = dt
        #Define PID controller with gains kp, ki, kd, gains in each array correspond to x,theta
        if self.use_visualization:
            viz = visualizer()

        #iterate through all steps to simulate the cartpole
        for i in range(steps):
            t = i * self.dt
            
            #update simulation one timestep. select controller in __init__
            self.u=self.controller.control(self.get_measured_state())

            max_control = 200
            if self.u > max_control:
                self.u = max_control
            if self.u < -max_control:
                self.u = -max_control

            self.update(self.u, dt)

            if self.use_visualization:
                viz.update(self.x, self.theta, self.L, self.u)

            #append, time, pose, and control data for plotting
            self.t_data.append(t)
            self.x_data.append(self.x)
            self.xd_data.append(self.xd)
            self.theta_data.append(self.theta * 180.0 / math.pi)
            self.u_data.append(self.u)

            if self.noisy:
                self.x_noisy_data.append(self.x_noisy)
                self.xd_noisy_data.append(self.xd_noisy)
                self.theta_noisy_data.append(self.theta_noisy * 180.0 / math.pi)
                self.thetad_noisy_data.append(self.thetad_noisy)

        if self.use_visualization:
            viz.end()

    def get_measured_state(self):
        if self.noisy:
            return np.array([self.x_noisy, self.xd_noisy, self.theta_noisy, self.thetad_noisy])
        
        else:
            return np.array([self.x, self.xd, self.theta, self.theta_d])



