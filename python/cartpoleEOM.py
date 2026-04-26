#A simulation to test different contoller for the cartpole controls problem.
#The objective is to balance a pendulum upright by applying a horizontal force to the cart is is mounted to
# The cart is able to move +/- 1 m from its initial position, and should actively try to center in the arena



import math
import numpy as np
import matplotlib.pyplot as plt
from control import bang_bang, PID
import random

g = 9.81 #m/s2

class Cartpole:

    def __init__(self, m1, m2, L):

        #specify which controller to use
        self.use_controller = "PID"


        #available controllers:
        controllers = ["none","bangbang", "PID"]

        #fetch system parameters
        self.m1 = m1
        self.m2 = m2
        self.L = L

        #initialize angle to random value in range
        #self.theta = random.uniform(-3.14, 3.14)
        self.theta = 0.5
        #initialize all other state variables to 0
        self.theta_d = 0
        self.theta_dd = 0
        self.x = 0.0
        self.xd = 0
        self.xdd = 0

        #create np arrays for each pose element
        self.acc = np.array([self.xdd, self.theta_dd])
        self.vel = np.array([self.xd, self.theta_d])
        self.pos = np.array([self.x, self.theta])

        #create empty arrays for plotting
        self.t_data = []
        self.x_data = []
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

        #Limit the cartpole inside the arena
        if self.pos[0] > 1.0:
            self.pos[0] = 1.0
        elif self.pos[0] < -1.0:
            self.pos[0] = -1.0


        self.x, self.theta = self.pos
        self.xd, self.theta_d = self.vel
        self.xdd, self.theta_dd = self.acc


        return 0
    
    #Plot pose using matplotlib
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
        
    #simulate the cartpole for number of steps with dt time increment
    def simulate(self, dt, steps):

        #Define PID controller with gains kp, ki, kd
        #gains in each array correspond to x,theta
        pid_controller = PID([-2.0, 1.0],[0.0,0.0],[0.0,0.0])

        #iterate through all steps to simulate the cartpole
        for i in range(steps):
            t = i * dt
            
            #update simulation one timestep. select controller defined in __init__
            if self.use_controller == "PID":
                self.update(pid_controller.control([self.x, self.theta, self.xd, self.theta_d], dt), dt)
            if self.use_controller == "bangbang":
                self.update(bang_bang(self.pos, 0.15, 1.0), dt)
            if self.use_controller == "none":
                self.update(0.0, dt)

            #add time and pose data for plotting
            self.t_data.append(t)
            self.x_data.append(self.x)
            self.theta_data.append(self.theta)


        
        return 0

#create cartpole with m1 = m2 = 0.5, L = 0.5
cartpole = Cartpole(1.0,0.5,0.5)

#Simulate 5000 timesteps with dt = 0.001
cartpole.simulate(0.001, 3000)
cartpole.plot_results()


