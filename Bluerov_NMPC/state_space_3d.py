import numpy as np
import geomutils as geom
from numpy.linalg import inv
from math import cos, sin


I3 = np.identity(3)
zero3= 0*I3
g = 9.81

# AUV parameters
m = 14.8 #kg 11.5
W = m*g      #N
_B = W+2     #N 
d = 0.15     #meters
r = d/2      #meters
L = 1.08     #meters
z_G = 0.2    #meters
r_G = [0,0,z_G]   #meters
thrust_min = -20  #N
thrust_max = 20   #N
U_max = 2         #m/s


# Moments of inertia
I_x =  0.16
I_y =  0.16
I_z =  0.16


Ig = np.vstack([
     np.hstack([I_x, 0, 0]),
     np.hstack([0, I_y, 0]),
     np.hstack([0, 0, I_z])])

X_udot =  -5.5
Y_vdot = -12.7
Z_wdot = -14.57
K_pdot = -0.12
M_qdot = -0.12
N_rdot = -0.12

# Linear damping parameters
X_u = -4.03
Y_v = -6.22
Z_w = -5.18
K_p = -0.07
M_q = -0.07
N_r = -0.07


# Nonlinear damping parameters，Here are the changes made to the description
X_uu = -18.18
Y_vv = -21.66
Z_ww = -36.99
K_pp = -1.55
M_qq = -1.55
N_rr = -1.55


# Lift parameters
C_LB = 1.24 #empirical body-lift coefficient
C_LF = 3    #empirical fin-lift coefficient
S_fin = 64e-4 #m^2
x_b = -0.4 #m
x_fin = -0.4 #m
rho = 1000 #kg/m^3

# Body Lift
Z_uwb = -0.5*rho*np.pi*(r**2)*C_LB
M_uwb = -(-0.65*L-x_b)*Z_uwb
Y_uvb = Z_uwb
N_uvb = -M_uwb

# Fin lift in Y and N 
Y_uvf = rho*C_LF*S_fin*(-1)
N_uvf = x_fin*Y_uvf
Y_urf = rho*C_LF*S_fin*(-x_fin)
N_urf = x_fin*Y_urf
Y_uudr = rho*C_LF*S_fin
N_uudr = x_fin*Y_uudr

# Fin lift in Z and M
Z_uwf = -rho*C_LF*S_fin
M_uwf = -x_fin*Z_uwf
Z_uqf = -rho*C_LF*S_fin*(-x_fin)
M_uqf = -x_fin*Z_uqf
Z_uuds = -rho*C_LF*S_fin
M_uuds = -x_fin*Z_uuds

# AUV Matrix of dynamics and kinematics equations
def M_RB():

    M_RB1 = np.array([[0, m*z_G, 0],
                     [-m*z_G, 0, 0],
                     [0,      0, 0]])
    M_RB2 = np.array([[0, -m*z_G, 0],
                      [m*z_G, 0, 0],
                      [0,      0, 0]])
    M_RB_CG = np.vstack([
        np.hstack([m*I3, M_RB1]),
        np.hstack([M_RB2, Ig])
    ])
    return M_RB_CG

def M_A():
    M_A = -np.diag([X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot])
    return M_A

def M_inv():
    M = M_RB() + M_A()
    return inv(M)

def C_RB(nu):
    w = nu[2]
    p = nu[3]
    q = nu[4]
    r = nu[5]
    v = nu[1]
    u = nu[0]
    C_RB1 = np.zeros((3,3))
    C_RB2 = np.array([[0, m*w, -m*v],
                      [-m*w, 0, m*u],
                      [m*v, -m*u, 0]])
    C_RB3 = np.zeros((3,3))
    # C_RB3 = np.array([[0, I_z*r, -I_y*q],
    #                   [-I_z*r, 0, I_x*p],                       
    #                   [I_y*q, -I_x*p, 0]])
    C_RB_CO = np.vstack([
                        np.hstack([C_RB1, C_RB2]),
                        np.hstack([C_RB2, C_RB3])
                        ])
    return C_RB_CO


def C_A(nu):
    u = nu[0]
    v = nu[1]
    w = nu[2]
    p = nu[3]
    q = nu[4]
    r = nu[5]

    C_11 = np.zeros((3,3))
    C_12 = np.array([[0, -Z_wdot*w, Y_vdot*v],
                     [Z_wdot*w, 0, -X_udot*u],
                     [-Y_vdot*v, X_udot*u, 0]])
    # C_12 = np.array([[0, Z_wdot*w, Y_vdot*v],
    #                 [-Z_wdot*w, 0, -X_udot*u],
    #                 [-Y_vdot*v, X_udot*u, 0]])

    C_21 = np.array([[0, -Z_wdot*w, Y_vdot*v],
                     [Z_wdot*w, 0, -X_udot*u],
                     [-Y_vdot*v, X_udot*u, 0]])
    C_22 = np.zeros((3,3))
    # C_22 = np.array([[0, -N_rdot*r, M_qdot*q],
    #                  [N_rdot*r, 0, -K_pdot*p],
    #                  [-M_qdot*q, K_pdot*p, 0]])
    
    C_A = np.vstack([np.hstack([C_11, C_12]), np.hstack([C_21, C_22])])
    return C_A


def C(nu):
    C = C_RB(nu) + C_A(nu)
    return C


def D(nu):
    u = abs(nu[0])
    v = abs(nu[1])
    w = abs(nu[2])
    p = abs(nu[3])
    q = abs(nu[4])
    r = abs(nu[5])

    D = -np.array([[X_u, 0, 0, 0, 0, 0],
                   [0, Y_v, 0, 0, 0, 0],
                   [0, 0, Z_w, 0, 0, 0],
                   [0, 0, 0, K_p, 0, 0],
                   [0, 0, 0, 0, M_q, 0],
                   [0, 0, 0, 0, 0, N_r]])
    D_n =-np.array([[X_uu*abs(u), 0, 0, 0, 0, 0],
                     [0, Y_vv*abs(v), 0, 0, 0, 0],
                     [0, 0, Z_ww*abs(w), 0, 0, 0],
                     [0, 0, 0, K_pp*abs(p), 0, 0],
                     [0, 0, 0, 0, M_qq*abs(q), 0],
                     [0, 0, 0, 0, 0, N_rr*abs(r)]])

    return D + D_n


def B(nu):
    B = np.array([
            [0.707, 0.707, -0.707, -0.707, 0, 0],                  
            [-0.707, 0.707, -0.707, 0.707, 0, 0],
            [0,       0,      0,     0,    1, 1],
            [0.051,  -0.051, 0.051, -0.051, 0.111, -0.111],
            [0.051,  0.051, -0.051, -0.051, 0.002, -0.002],
            [-0.167, 0.167,  0.175, -0.175, 0, 0],
        ])

    return B
def G(eta):
    phi = eta[3]
    theta = eta[4]
    G = np.array([0,
                  0,
                  -(W-_B)*cos(theta)*cos(phi),
                  0,
                  0,
                  0])
    return G
