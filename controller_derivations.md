This md file details the math behind the controllers used in this project, and also includes some useful practical and implementation considerations.

# PID Control

PID control is one of the simplest control systems available. The advantages of PID control are that it does not require a model of the system, and is extremely simple to implement, often in less than 5 lines of code. The lack of a model can also be a disadvantage though, as small changes in system parameters can often make the PID controller unusable and need to be retuned. 

PID control is entirely based on the error $e$ of the system - how far some measured quantity is from a target value. The control $u$ from a PID controller is computed as:

$u = k_p  e +  k_i  \int edt + k_d  \dot{e} $

where $k_p$, $k_i$, $k_d$ are user-tuned gains. 

Conceptually, the $k_p$ (proportional) term measures how far the current state of the system is from the target state. The $k_i$ (integral) term sums accumulated error over time, which is useful for eliminating steady-state error or overcoming actuator deadzones. The $k_d$ (derivative) term acts as a damper to reduce overshooting the target state. 

To tune the gains, a useful procedure is often to increase the $k_p$ gain until the response is acceptable, then tune the $k_d$ gain until oscillations are removed or decreased enough. If a steady-state error remains, then the $k_i$ gain should be tuned. 

# Cartpole Equations of Motion

The following controllers all require information about the system being controlled. The equations of motion (EOM) describe the motion of the system as a result of the forces acting on it. For this derivation the downward direction is defined as negative and a positive angle is measured counterclockwise starting at the upward vertical. 

In matrix form the equations of motion for the cartpole system are:

<img width="513" height="60" alt="image" src="https://github.com/user-attachments/assets/ac08d2b9-576e-4e51-bfb4-48ea39ca4f20" />

See below for the derivation. 

### Kinematics

The kinematics of the system describe its motion without referencing forces. The position of the pendulum, $(x_p, y_p)$, relative to the global origin is defined as:

$x_p = x + sin(\theta) L$                                                                       $\hspace{5cm} y_p = Lcos(\theta)$

$\dot{x_p} = \dot{x} + \dot{\theta}Lcos(\theta)$                                                $\hspace{4.25cm} \dot{y_p}=-L\dot{\theta}sin(\theta)$

$\ddot{x_p} = \ddot{x} + L(\ddot{\theta}cos(\theta) - \dot{\theta}^2 sin(\theta))$                $\hspace{2.15cm} \ddot{y_p} = -L(\ddot{\theta}sin(\theta)+\ddot{\theta}cos(\theta))$

Where $x$ is the cart position relative to the global origin. 

### Pendulum EOM

The pendulum is assumed to be massless except for a point-mass of mass $m_p$ at length $L$. When the pendulum is at angle $\theta$, the forces acting on the pendulum are:

$F_x = -T sin(\theta) $

$F_y = -T cos(\theta) -mg $

Where $T$ is the tension force in the pendulum.

Applying $F=ma$ to relate the forces to the kinematics gives:

$m_p \ddot{x_p}=-Tsin(\theta)$

$m_p \ddot{y_p} = -Tcos(\theta)-m_p g $

Substituting the equations for the kinematics of the pendulum and then multiplying the $x$ equation by $cos(\theta)$ and the $y$ equation by $sin(\theta)$, then adding the resulting equations together yields the equation of motion for the pendulum:

$m_p cos(\theta) \ddot{x} + m_p L \ddot{\theta} = m_p g sin(\theta) $

### Cart EOM

The EOM for the cart can be obtained directly from $F=ma$, where $x$ is the cart position and $u$ is the applied control input:

$m \ddot{x} = u + T sin(\theta)$

Substituting the equation from the pendulum derivation for $T sin(\theta)$ results in:

$m \ddot{x} = u - m_p\ddot{x} - m_p L \ddot{\theta} cos(\theta) + L \dot{\theta}^2 sin(\theta) m_p$

After some algebraic manipulation, it becomes:

$(m+m_p) \ddot{x} + m_p L \ddot{\theta} cos(\theta) = u + L \dot{\theta}^2 sin(\theta) $

#Linearized Equation of Motion

As the pendulum system generally operates with small values of $\theta$, the equations of motion can be linearized to simplify calculation and enable more responsive control from the optimal controllers. To linearize the system, the small angle approximation can be used to set $cos(\theta) = 1 $ and $sin(\theta) = \theta$. Additionally, the nonlinear $\dot{\theta}$ term can also be assumed to be $0$. 

The linearizaton results in:

<img width="308" height="58" alt="image" src="https://github.com/user-attachments/assets/797e79c9-c070-49e0-b9b4-f5ac2ea227bf" />

Solving for the acceleration vector gives:

<img width="192" height="72" alt="image" src="https://github.com/user-attachments/assets/890049fb-e7f0-44e8-a427-362565076b82" />

It is then useful to write the equations in the standard $\dot{x} = Ax + Bu$ state space form:

<img width="401" height="129" alt="image" src="https://github.com/user-attachments/assets/32a9ff42-05c3-4838-8be5-f9dbe1b078dd" /> 

with the state $\bf{x}$ defines as :

<img width="84" height="115" alt="image" src="https://github.com/user-attachments/assets/d92cb39e-5345-46c7-bda1-065ce15e8f31" />




# LQR Control

Linear Quadratic Regulator (LQR) control is an advancement from PID control as the LQR controller considers a dynamical model of the system. Additionally, the LQR controller works not only to minimize error in the system, but also to minimize control effort. This can be useful to avoid actuator saturation, and to save energy during deployment. 

The LQR controller is built around this cost function:

$J = \int (x^T Q x + u^TRu )dt$

where $x$ is the state of the system, $Q$ is a state weighting matrix and $R$ is an control weighting matrix. The $Q$ and $R$ matricies can be tuned by the user to define desired system properties. The goal is to find a control input $u$ that minimizes the overall cost over all time. 

This is accomplished by also minimizing the future cost $V$.

Definining $V(x)= \min_{u} J$

The Bellman Optimality Principle states that for a policy to be optimal, it must be optimal in the future despite the current/initial conditions. To expand $V(x)$ to include this, we can rewrite it as:

$V(x) = min_{u} (x^T Q x + u^T R u)dt + V(x+\dot{x}dt)$

We can then express the $V(x+\dot{x}dt)$ term with a first order taylor expansion:

$V(x+\dot{x}dt) \approx V(x) + \frac{\partial{V}}{\partial{x}} \dot{x} dt$ 

Rewriting the equation for $V(x)$ using the taylor expansion gives:

$V(x) = min_{u} (x^T Q x + u^T R u)dt + V(x) + \frac{\partial{V}}{\partial{x}} \dot{x} dt$

Simplifying algebraicly results in:

$0 = min_{u} (x^T Q x + u^T R u +  \frac{\partial{V}}{\partial{x}} \dot{x})$

The state space equation of the system $\dot{x} = Ax + Bu$ can then be substituted into this equation to result in:

$0 = min_{u} (x^T Q x + u^T R u +  \frac{\partial{V}}{\partial{x}}( Ax + Bu))$

This equation is a form of the [Hamilton-Jacobi-Bellman (HJB) equation](https://en.wikipedia.org/wiki/Hamilton%E2%80%93Jacobi%E2%80%93Bellman_equation).

To solve it, we can assume $V = x^T P x$, where $P$ is a positive semi-definite matrix, then:

$\frac{\partial V}{\partial x} = 2 x^T P$

Substituting into the HJB and expanding all terms gives:

$0 = min_{u} (x^T Q x + u^T R u + 2 x^T P A x + 2 x^T P B u) $

To find the minimum of this, we can apply some simple calculus! We just want to find where the derivative with respect to u is 0! To make things easier, we can also ignore any terms that dont contain a $u$ variable.

$\frac{\partial}{\partial u} (u^T R u + 2 x^T P B u) = 0$

Evaluating this derivative gives us:

$2 R u + 2 B^T P x = 0$

Solving for $u$ then gives us:

$u = -R^-1 B^T P x$

Note that this is in the form of the optimal control law $u = -k x$, where $k = R^-1 B^T P$ !

To solve for the positive semi-definite matrix $P$, we can assemble the [Algebraic Riccati Equation](https://en.wikipedia.org/wiki/Algebraic_Riccati_equation) by substituting the solution we found for $u$ into the HJB equation, resulting in:

$0 = x^T (Q - PBR^-1 B^T P + PA + A^TP) x$

Note that this is the continuous form of the algebraic Riccati Equation (CARE). For discrete time implementations (like is used in this simulator) there also exists a Discrete Algebraic Riccati Equation (DARE). 

There are multiple ways to solve for the $P$ matrix, including convex optimization solvers (such as those provided by SciPy or cvxpy) and iterative methods (as was implemented in the C++ implementation of the LQR controller in this simulator). 





