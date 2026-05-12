import math
import numpy as np
from scipy.linalg import solve_continuous_are
import cvxpy as cp
from scipy.signal import cont2discrete

class PID:
    def __init__(self, kp, ki, kd):
        self.error = np.array([0.0,0.0])
        self.error_d = np.array([0.0,0.0])
        self.error_i = np.array([0.0,0.0])

        self.kp = np.array([kp[0], kp[1]])
        self.ki = np.array([ki[0], ki[1]])
        self.kd = np.array([kd[0], kd[1]])

    def __str__(self):
        return "PID"

    def control(self, state, dt=0.001):

        self.error = np.array([state[0], state[2]])
        self.error_d = np.array([state[1], state[3]])
        self.error_i = self.error_i + self.error * dt

        u = self.kp @ np.transpose(self.error) + self.ki @ np.transpose(self.error_i) + self.kd @ np.transpose(self.error_d)

        return u
    
class LQR:
    def __init__(self, A, B, Q, R):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        #Solve the Riccati equation
        self.P = solve_continuous_are(self.A, self.B, self.Q, self.R)

        #Compute gains
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P

    def __str__(self):
        return "LQR"
    
    def control(self, state, dt=0.001):
        #state should be a numpy array of form [x, xdot, theta, thetadot]
        u = -self.K @ state
        return float(u.item())

def create_LQR_mats(m1, m2, L):
    g = 9.81
    A = np.array([[0, 1, 0, 0],[0, 0, (-m2*g)/m1, 0],[0, 0, 0, 1],[0, 0, ((m1+m2)*g)/(m1*L), 0]])
    B = np.array([[0],[1/m1],[0],[-1/(m1*L)]])

    return A, B

def build_G(A, B, N):
        nx, nu = B.shape
        G = np.zeros((nx*N, nu*N))

        for i in range(N):
            for j in range(i+1):
                A_power = np.linalg.matrix_power(A, i-j)
                G[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = A_power @ B

        return G

class MPC:
    def __init__(self, A, B, Q, Qf, R, n=3, dt=0.001):

        self.n = n

        Ad, Bd, _, _, _ = cont2discrete((A, B, np.eye(4), np.zeros((4,1))),0.02)
        A_powers = [np.linalg.matrix_power(Ad,i) for i in range(1, self.n+1)]

        self.F = np.vstack(A_powers)
        self.G = build_G(Ad,Bd,n)

        self.Q_bar = np.kron(np.eye(self.n), Q)
        self.Q_bar[-4:, -4:] = Qf
        
        zr = np.zeros((1,1))
        self.R_bar = np.kron(np.eye(self.n), R)

        self.H = self.G.T @ self.Q_bar @ self.G + self.R_bar
        self.U = cp.Variable((self.n, 1))

        u_max = 200
        self.constraints = [
            self.U <= u_max,
            self.U >= -u_max
        ]

        self.f = cp.Parameter((self.n,1))
        self.cost = 0.5 * cp.quad_form(self.U, self.H) + self.f.T @ self.U

        self.problem = cp.Problem(cp.Minimize(self.cost), self.constraints)

    def __str__(self):
        return "MPC"
    
    def control(self, state, dt=0.001):
        
        f_t = (2 * state.T @ self.F.T @ self.Q_bar @ self.G).reshape(-1, 1)
        self.f.value = f_t

        self.problem.solve(solver=cp.OSQP, warm_start=True)

        U_opt = self.U.value
        u = U_opt[0, 0]
        return float(u)
            