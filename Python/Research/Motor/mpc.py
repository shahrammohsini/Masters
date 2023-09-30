import numpy as np
from scipy.signal import cont2discrete
import Optimizer as op
import control

#define constants for system identification model
Ymin = 0
y_meas = 0
# Define your continuous-time state-space matrices
# Define the constants for stat stapce model
J = 0.1
b = 0.5
Kt = 1
L = 1  # Note: You've defined L twice in your MATLAB code. Assuming the second definition is the correct one.
R = 5
Ke = 1

desired_velocity = 1000 #RPM

# Define the system matrices
A = np.array([[0, 1, 0],
              [0, -b/J, Kt/J],
              [0, -Ke/L, -R/L]])

B = np.array([[0],
              [0],
              [1/L]])

C = np.array([[0, 1, 0]])

D = np.array([0])


Ts = 0.01  # Define your sampling time
dt = Ts
# Discretize the matrices
system = (A, B, C, D)
A_d, B_d, C_d, D_d, _ = cont2discrete(system, Ts, method='zoh')






#Setup MPC
nx, nu = A_d.shape[0], B_d.shape[1]

# MPC setup
N = 10  # Prediction horizon
Q = np.eye(nx)  # State cost matrix
R = np.eye(nu)  # Control cost matrix

# QP matrices for the MPC problem
H = np.block([[np.kron(np.eye(N), Q), np.zeros((N*nx, N*nu))], 
              [np.zeros((N*nu, N*nx)), np.kron(np.eye(N), R)]])
f = np.zeros(N*(nx+nu))

# Constraint matrices for the system dynamics
M_con = np.zeros((N*nx, N*(nx+nu)))

for i in range(N):
    Ai = np.linalg.matrix_power(A_d, i+1)
    for j in range(i+1):
        M_con[i*nx:(i+1)*nx, j*nx:(j+1)*nx] = Ai
        Ai = Ai @ A_d
    M_con[i*nx:(i+1)*nx, N*nx + i*nu:N*nx + (i+1)*nu] = np.linalg.matrix_power(A_d, i) @ B_d


# Assuming no input constraints, and the states are fully measurable
def mpc_control(x0):
    b_eq = np.zeros((N*nx, 1))
    
    # Set desired trajectory for the velocity over the prediction horizon
    for i in range(N):
        b_eq[i*nx + 1] = desired_velocity  # Assuming the velocity is the second state
    


    # Use one of the provided solvers, e.g., HQP
    solver = op.HQP(M_con, np.linalg.inv(H), b_eq, f)
    solver.Optimize()

    # Extract the first control input from the solution
    u0_optimal = solver.lam[N*nx:N*nx+nu]

    return u0_optimal




#We need all three states but only measure one (velocity) so we'll use a state estimater to estimate the other two
# Desired poles for the estimation error dynamics
observer_poles = np.array([0, -25 + 15.81j, -25 - 15.81j])  # Example for a 3x3 A matrix

# Compute L using pole placement
L = control.place(A_d.T, C_d.T, observer_poles).T

# Initialize states and estimates
x_current = np.zeros((3, 1))
x_hat = np.zeros((3, 1))  # Initial state estimate



while True:
    print("u", u)
    u = mpc_control(x_hat)
    # Apply u to the motor and get the new state x_current
    k = 7.576*u[0, 0] + 400.05 # Better data from python k = 449.29
    tau = -0.0455*u[0, 0] + 1.2455 # Better data from python tau = 0.927
    #print(u)
    y_meas = ((k*u[0, 0]*dt + tau*Ymin)/ (tau + dt))+ np.random.rand(1)  # in discrete domain
    print("y", y_meas)
    # This might involve interfacing with hardware or a simulation environment


    # 3. Update the estimated state using the Luenberger observer
    y_hat = C @ x_hat
    x_hat = A @ x_hat + B @ u + L @ (y_meas.reshape(-1, 1) - y_hat)




    t = t + dt
    Ymin = y_meas[0]




