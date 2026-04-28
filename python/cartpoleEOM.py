#A simulation to test different contoller for the cartpole controls problem.
#The objective is to balance a pendulum upright by applying a horizontal force to the cart is is mounted to
# The cart is able to move +/- 1 m from its initial position, and should actively try to center in the arena
# A visualizer can be used by setting the parameter in init to True

import math
import numpy as np
import matplotlib.pyplot as plt
from control import bang_bang, PID
import random
from visualization import *

g = 9.81 #m/s2

class Cartpole:

    def __init__(self, m1, m2, L):

        #PARAMETERS
        self.use_controller = "PID"   #available controllers: ["none","bangbang", "PID"]
        self.use_visualization = True
        
        #Controller gains [x,theta]:
        self.kp = [14.5, 50.0]
        self.ki = [0.0,2.0]
        self.kd = [9.0,9.0]

        #fetch system parameters
        self.m1 = m1
        self.m2 = m2
        self.L = L

        #initialize angle to random value in range
        self.theta = random.uniform(-0.5, 0.5)
        self.theta0 = self.theta
        print(f"starting angle is {self.theta} rads, {self.theta * 180.0 / math.pi} degs")

        #initialize all other state variables to 0
        self.theta_d = 0.0
        self.theta_dd = 0.0
        self.x = 0.0
        self.xd = 0.0
        self.xdd = 0.0

        self.u = 0.0

        #create np arrays for each pose element
        self.acc = np.array([self.xdd, self.theta_dd])
        self.vel = np.array([self.xd, self.theta_d])
        self.pos = np.array([self.x, self.theta])

        #create empty arrays for plotting
        self.t_data = []
        self.x_data = []
        self.u_data = []
        self.theta_data = []

    def update(self, u,t):
        #update the state of the cartpole system after a short timestep t
        c = math.cos(self.theta)
        s = math.sin(self.theta)

        #EOM of form M_mat * [xdd, theta_dd] = F
        self.M_mat = np.array([
            [self.m1 + self.m2, self.m2 * self.L * c],
            [c, self.L]
        ])

        self.F = np.array([
            u + self.m2 * self.L * (self.theta_d**2) * s,
            g * s   
        ])

        #Solve for the current linear and angular acceleration
        self.acc = np.linalg.solve(self.M_mat, self.F) 

        #Update using EOM
        self.vel = self.vel + self.acc * t
        self.pos = self.pos + self.vel * t


        self.x, self.theta = self.pos
        self.xd, self.theta_d = self.vel
        self.xdd, self.theta_dd = self.acc

    
    #Plot pose using matplotlib
    def plot_results(self):
        plt.figure()

        plt.subplot(3,1,1)
        plt.plot(self.t_data, self.x_data)
        plt.ylabel("Cart Position x (m)")
        plt.grid()

        plt.subplot(3,1,2)
        plt.plot(self.t_data, self.theta_data)
        plt.ylabel("Pole Angle θ (degrees)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.subplot(3,1,3)
        plt.plot(self.t_data, self.u_data)
        plt.ylabel("Control Input u (N)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.tight_layout()
        plt.show()

        
    def create_metrics(self,dt):
        print("=========================================")
        print("Metrics")
        print("=========================================")

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
        for i, u in enumerate(self.u_data):
            self.u_RMS += u ** 2
        self.u_RMS = math.sqrt(self.u_RMS / (i+1))
        
        print(f"RMS Control input: {self.u_RMS} N")
        print(f"Maximum Control Input {max(self.u_data)}")        

    def simulate(self, dt, steps):

        #Define PID controller with gains kp, ki, kd, gains in each array correspond to x,theta
        pid_controller = PID(self.kp,self.ki,self.kd)
        
        if self.use_visualization:
            viz = visualizer()

        #iterate through all steps to simulate the cartpole
        for i in range(steps):
            t = i * dt
            
            #update simulation one timestep. select controller in __init__
            if self.use_controller == "PID":
                self.u = pid_controller.control([self.x, self.theta, self.xd, self.theta_d], dt)
                
            if self.use_controller == "bangbang":
                self.u = bang_bang(self.pos, 0.15, 1.0)
            if self.use_controller == "none":
                self.u = 0.0

            if self.u > 100:
                self.u = 100
            if self.u < -100:
                self.u = -100

            self.update(self.u, dt)

            if self.use_visualization:
                viz.update(self.x, self.theta, self.L, self.u)

            #append, time, pose, and control data for plotting
            self.t_data.append(t)
            self.x_data.append(self.x)
            self.theta_data.append(self.theta * 180.0 / math.pi)
            self.u_data.append(self.u)

        if self.use_visualization:
            viz.end()

#Create cartpole object with arguments m1, m2, L
cartpole = Cartpole(1.0,0.5,0.5)

#Simulate 5000 timesteps with dt = 0.001
dt = 0.001
cartpole.simulate(dt, 5000)
cartpole.create_metrics(dt)
cartpole.plot_results()


