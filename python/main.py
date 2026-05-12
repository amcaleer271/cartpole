from cartpoleEOM import Cartpole
from control import *
import numpy as np

m1 = 1.0
m2 = 0.5
L = 0.5
A, B = create_LQR_mats(m1, m2, L)
dt = 0.001

#MPC

Q_mat = np.diag([10.0, 10.0, 20.0, 5.0])
Q_f = np.diag([50.0, 5.0, 100.0, 2.0])

controller = MPC(A,B,Q_mat,Q_f,np.array([[0.5]]),n=100, dt=0.02)

MPC_cartpole = Cartpole(m1, m2, L, controller, noisy=False, theta0=0.25)
MPC_cartpole.simulate(dt, 5000)
MPC_cartpole.create_metrics()
MPC_cartpole.plot_results()


#PID 
controller = PID([14.5, 50.0],[0.0,2.0],[9.0,9.0])

PID_cartpole = Cartpole(m1, m2, L, controller, noisy=False, theta0=0.25)
PID_cartpole.simulate(0.001, 5000)
PID_cartpole.create_metrics()
PID_cartpole.plot_results()

#LQR 

controller = LQR(A,B,np.diag([15.0, 3.0, 30.0, 4.0]),np.array([[0.5]]) )

LQR_cartpole = Cartpole(m1, m2, L, controller, noisy=False ,theta0=0.25)
LQR_cartpole.simulate(0.001, 5000)
LQR_cartpole.create_metrics()
LQR_cartpole.plot_results()
