This md file details the math behind the controllers used in this project, and also includes some useful practical and implementation considerations.

# PID Control

PID control is one of the simplest control systems available. The advantages of PID control are that it does not require a model of the system, and is extremely simple to implement, often in less than 5 lines of code. The lack of a model can also be a disadvantage though, as small changes in system parameters can often make the PID controller unusable and need to be retuned. 

PID control is entirely based on the error $e$ of the system - how far some measured quantity is from a target value. The control $u$ from a PID controller is computed as:

$u = k_p  e +  k_i  \int edt + k_d  \dot{e} $  

where $k_p$, $k_i$, $k_d$ are user-tuned gains. 

Conceptually, the $k_p$ (proportional) term measures how far the current state of the system is from the target state. The $k_i$ (integral) term sums accumulated error over time, which is useful for eliminating steady-state error or overcoming actuator deadzones. The $k_d$ (derivative) term acts as a damper to reduce overshooting the target state. 

To tune the gains, a useful procedure is often to increase the $k_p$ gain until the response is acceptable, then tune the $k_d$ gain until oscillations are removed or decreased enough. If a steady-state error remains, then the $k_i$ gain should be tuned. 

# LQR Control

Linear Quadratic Regulator (LQR) control is an advancement from PID control as the LQR controller considers a dynamical model of the system. Additionally, the LQR controller works not only to minimize error in the system, but also to minimize control effort. This can be useful to avoid actuator saturation, and to save energy during deployment. 

The LQR controller is built around the cost function $J$ given in (2).

$J = \int (x^T Q x + u^TRu )dt$

where $x$ is the state of the system, $Q$ is a state weighting matrix and $R$ is an control weighting matrix. By tuning the $Q$ and $R$ matricies, both the state of the system and the control input can be minimized. 
