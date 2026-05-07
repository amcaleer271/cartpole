"""
This file contains the Cartpole class and its methods. 
The Cartpole object can be used to simulate a simple cartpole via its equations of motion (EOM)

The included methods are:
update . . . . . . . . advances the simulation one timestep forward
plot_results . . . . . creates plots of the position, angle, and control input of the simulation
create_metrics . . . . provides info about the system including settling time, initial conditions, and results
simulate . . . . . . . runs a loop for a provided number of iterations to run a complete simulation using the update() method
get_measured_state . . returns the current state of the system in numpy array [x, x_dot, theta, theta_dot]

When initializing the Cartpole class, the arguments are:
(required) m1 . . . . . mass of the cart . . . . . . . . . . . . . . . . . . . . . . . . (float, kg)
(required) m2 . . . . . mass of the pendulum . . . . . . . . . . . . . . . . . . . . . . (float, kg)
(required) L . . . . . .length of the pendulum . . . . . . . . . . . . . . . . . . . . . (float, m)
(required) controller . selected controller. . . . . . . . . . . . . . . . . . . . . . . (controller object)
(optional) use_vis . . .whether to use a 2D visualization . . . . . . . . . . . . . . . .(bool), default = False
(optional) theta0 . . . starting angle of the pendulum . . . . . . . . . . . . . . . . . (float, rads), default = random.uniform(-0.5,0.5)
(optional) noisy . . . .whether the sensor measurements should be simulated to be noisy. (bool), default = False

Physics/coordinate system notes:
x = 0 is the center of the track
theta = 0 if the pendulum completely upright
Positive theta is counterclockwise
Positive control u is a force applied on the cart to the right

Example usage with LQR controller:

A, B = create_LQR_mats(1.0, 0.5, 0.5)
controller = LQR(A,B,np.diag([15.0, 3.0, 30.0, 4.0]),np.array([[0.5]]) )

LQR_cartpole = Cartpole(1.0, 0.5, 0.5, controller, noisy=True ,theta0=0.25)
LQR_cartpole.simulate(0.001, 5000)
LQR_cartpole.create_metrics()
LQR_cartpole.plot_results()

Adam McAleer
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from control import PID, LQR, create_LQR_mats
import random
from visualization import *
from lkf import LinearKalmanFilter

g = 9.81 #gravity constant, m/s2

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
        self.prev_u = 0.0

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
            self.x_kf_prediction = []
            self.theta_kf_prediction = []

    def update(self, u, t) -> None:
        """
        The update function progresses the simulation by one timestep according to the equation of motion
        
        :param u: The current control input u (float, N)
        :param t: The timestep to progress the simulation (float, s)

        :updates:
        self.x, self.theta, self.xd, self.theta_d, self.xdd, self.theta_dd

        :returns: nothing
        """

        c = math.cos(self.theta)
        s = math.sin(self.theta)

        #cartpole EOM in form M_mat * [xdd, theta_dd] = F
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

        #Update state using acceleration from EOM
        self.vel = self.vel + self.acc * t
        self.pos = self.pos + self.vel * t

        #Store the resulting state information
        self.x, self.theta = self.pos
        self.xd, self.theta_d = self.vel
        self.xdd, self.theta_dd = self.acc

        #Simulate sensor noise by adding random gaussian values to the position and angle estimates
        if self.noisy:
            self.x_noisy = self.x + random.gauss(0.0, 0.25)
            self.theta_noisy = self.theta + random.gauss(0.0, 0.25)
            self.xd_noisy = self.xd
            self.thetad_noisy = self.theta_d 
    
    
    def plot_results(self) -> None:
        """
        The plot_results function creates a set of plots with the position, angle, and control input of the simulation
        
        no parameters

        updates no values

        returns nothing
        """
        
        #Create a figure titled with the current controller
        plt.figure()
        plt.suptitle(f"{self.controller}")

        #Create the first column of plots showing the true position, angle, and control input of the cartpole
        plt.subplot(7,3,1)
        plt.plot(self.t_data, self.x_data)
        plt.ylabel("Cart Position x (m)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.subplot(7,3,4)
        plt.plot(self.t_data, self.theta_data)
        plt.ylabel("Pole Angle θ (deg)")
        plt.xlabel("Time (s)")
        plt.grid()

        plt.subplot(7,3,7)
        plt.plot(self.t_data, self.u_data)
        plt.ylabel("Control Input u (N)")
        plt.xlabel("Time (s)")
        plt.grid()

        
        if self.noisy:

            #Create a second column of plots showing the measured (noisy) position and angle
            plt.subplot(7,3,2)
            plt.plot(self.t_data, self.x_noisy_data)
            plt.ylabel("Measured Cart Position x (m)")
            plt.xlabel("Time (s)")
            plt.grid()

            plt.subplot(7,3,5)
            plt.plot(self.t_data, self.theta_noisy_data)
            plt.ylabel("Measured Pole Angle θ (deg)")
            plt.xlabel("Time (s)")
            plt.grid()

            #Create a third column of plots showing the estimate from the kalman filter
            plt.subplot(7,3,3)
            plt.plot(self.t_data, self.x_kf_prediction)
            plt.ylabel("Estimated Cart Position x (m)")
            plt.xlabel("Time (s)")
            plt.grid()

            plt.subplot(7,3,6)
            plt.plot(self.t_data, self.theta_kf_prediction)
            plt.ylabel("Estimated Pole Angle θ (deg)")
            plt.xlabel("Time (s)")
            plt.grid()

        plt.tight_layout()
        plt.show()

        
    def create_metrics(self) -> None:
        """
        The create_metrics function prints useful metrics and conditions to the terminal
        
        no parameters

        updates no values

        returns nothing
        """

        print("=========================================")
        print(f"{self.controller} Metrics")
        print("=========================================")

        #Print the initial conditions
        print("----------Setup----------")
        print(f"Mass 1 = {self.m1}")
        print(f"Mass 2 = {self.m2}")
        print(f"Length = {self.L}")

        #Settling time and final value - time until response remains within 5% of initial value
        print("----------Settling----------")
        target_value = 0.05 * self.theta0
        print(f"Target (5% of intial value): {target_value}")
        print(f"Final value: {self.theta_data[-1]}")

        #If the final value is greater than the target, the system did not settle
        if abs(self.theta_data[-1]) > abs(target_value):
            print(f"System did not reach < 5% settling")

        #Else, find the last value where the system was outside the settling value, the time index of this is the settling time
        else:
            self.rev_theta = self.theta_data[::-1]
            self.settled_index = len(self.rev_theta) - next((i for i, x in enumerate(self.rev_theta) if abs(x) > abs(0.05 * self.theta0)), None)
            print(f"System settled after {self.t_data[self.settled_index]} s")

        #Metrics for controller input and performance
        print("----------Control----------")
        self.u_RMS = 0.0
        self.work = 0.0
        for i, u in enumerate(self.u_data):
            self.u_RMS += u ** 2
            self.work += abs(u * self.xd_data[i]) * self.dt
        self.u_RMS = math.sqrt(self.u_RMS / (i+1))
        
        print(f"RMS Control input: {self.u_RMS} N")
        print(f"Maximum Control Input {max(self.u_data)}")        
        print(f"Work done: {self.work} J")    

    def simulate(self, dt, steps) -> None:
        """
        The simulate function runs the entire simulation loop
        
        :param dt: How much time should pass between update loops (float, s)
        :param steps: How many iterations the simulation should loop (int)

        :updates: 
        self.x_data, self.theta_data, self.xd_data, self.u_data

        Note: the angle in self.theta_data is in degrees

        :returns: nothing
        """

        self.dt = dt

        #Create an instance of the visualizer
        if self.use_visualization:
            viz = visualizer()

        #Get the linearized state matricies and create an instance of the kalman filter
        if self.noisy:
            A, B = create_LQR_mats(1.0, 0.5, 0.5)
            lkf = LinearKalmanFilter(A, B, self.dt)

        #iterate through all steps to simulate the cartpole
        for i in range(steps):
            t = i * self.dt
            
            if not self.noisy:
                self.u=self.controller.control(self.get_measured_state())
            else:
                x_hat = self.get_measured_state()              #Current state of the system
                z = np.array([x_hat[0], x_hat[2]])             #Measurement for kalman filter
                kf_estimate = lkf.estimate(self.prev_u, z)     #State estimate from kalman filter
                self.u = self.controller.control(kf_estimate)  #Control input based on kf state estimate

            #Cap the max input (N) the contoller can provide
            max_control = 200   
            if self.u > max_control:
                self.u = max_control
            if self.u < -max_control:
                self.u = -max_control

            #Progress the simulation one timestep
            self.update(self.u, dt)

            #Store the control input for use in kalman filter
            self.prev_u = self.u

            #Progress the visualization by one frame if using it
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
                self.theta_noisy_data.append(self.theta_noisy * 180.0 / math.pi)
                self.x_kf_prediction.append(kf_estimate[0])
                self.theta_kf_prediction.append(kf_estimate[2] * 180.0 / math.pi)

        #End the visualization once complete
        if self.use_visualization:
            viz.end()

    def get_measured_state(self) -> np.array:
        """
        The get_measured_state function returns the current state of the system
        
        no params

        no updates

        :returns: np array of state in form [x, xd, theta, theta_d]
        """
        if self.noisy:
            return np.array([self.x_noisy, self.xd_noisy, self.theta_noisy, self.thetad_noisy])
        
        else:
            return np.array([self.x, self.xd, self.theta, self.theta_d])



