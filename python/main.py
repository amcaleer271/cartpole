from cartpoleEOM import Cartpole
from control import *
import numpy as np
"""
#PID 
controller = PID([14.5, 50.0],[0.0,2.0],[9.0,9.0])

PID_cartpole = Cartpole(1.0, 0.5, 0.5, controller, noisy=True)
PID_cartpole.simulate(0.001, 5000)
PID_cartpole.create_metrics()
PID_cartpole.plot_results()
"""
#LQR 
A, B = create_LQR_mats(1.0, 0.5, 0.5)
controller = LQR(A,B,np.diag([15.0, 3.0, 30.0, 4.0]),np.array([[0.5]]) )

LQR_cartpole = Cartpole(1.0, 0.5, 0.5, controller, noisy=True ,theta0=0.25)
LQR_cartpole.simulate(0.001, 5000)
LQR_cartpole.create_metrics()
LQR_cartpole.plot_results()
