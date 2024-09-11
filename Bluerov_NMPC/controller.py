import os
import casadi as cs
import numpy as np
import math
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from numpy import cos, sin, tan
from quadrotor import Quadrotor3D
from utils import skew_symmetric, v_dot_q, quaternion_inverse
from casadi import MX


dt = 0.05
class Controller:
    def __init__(self, quad:Quadrotor3D, t_horizon=1, n_nodes=20,
                 q_cost=None, r_cost=None, q_mask=None, rdrv_d_mat=None,
                 model_name="quad_3d_acados_mpc", solver_options=None):
        """
        :param quad: quadrotor object
        :type quad: Quadrotor3D
        :param t_horizon: time horizon for MPC optimization
        :param n_nodes: number of optimization nodes until time horizon
        :param q_cost: diagonal of Q matrix for LQR cost of MPC cost function. Must be a numpy array of length 12.
        :param r_cost: diagonal of R matrix for LQR cost of MPC cost function. Must be a numpy array of length 4.
        :param q_mask: Optional boolean mask that determines which variables from the state compute towards the cost
        function. In case no argument is passed, all variables are weighted.
        :param solver_options: Optional set of extra options dictionary for solvers.
        :param rdrv_d_mat: 3x3 matrix that corrects the drag with a linear model according to Faessler et al. 2018. None
        if not used
        """

        # Weighted squared error loss function q = (p_xyz, a_xyz, v_xyz, r_xyz), r = (u1, u2, u3, u4) 
        if q_cost is None:
            q_cost = np.array([10, 10, 10, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
            #q_cost = np.array([10, 10, 10, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0])
        if r_cost is None:
            r_cost = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        r_dot_cost = np.array([1] * 6)     # The thruster rate-of-change weights according to the actual situation.
        self.T = t_horizon                 # Time horizon
        self.N = n_nodes                   # number of control nodes within horizon

        self.quad = quad

        self.max_u = quad.max_input_value
        self.min_u = quad.min_input_value

        # Declare model variables
        self.postion = cs.MX.sym('p', 3)  # position x--y-z
        self.q = cs.MX.sym('a', 3)        # angle quaternion (wxyz) p - q - r 
        self.v = cs.MX.sym('v', 3)        # velocity                u - v -r    
        self.r = cs.MX.sym('r', 3)        # angle rate 
        
        # Full state vector (12-dimensional)
        self.x = cs.vertcat(self.postion, self.q, self.v, self.r)
        self.state_dim = 12

        # Control input vector, set the six thrusters.
        u1 = cs.MX.sym('u1')
        u2 = cs.MX.sym('u2')
        u3 = cs.MX.sym('u3')
        u4 = cs.MX.sym('u4')
        u5 = cs.MX.sym('u5')
        u6 = cs.MX.sym('u6')
        self.u = cs.vertcat(u1, u2, u3, u4, u5, u6)

        # Add input rate 
        delta_u1 = cs.MX.sym('delta_u1')
        delta_u2 = cs.MX.sym('delta_u2')
        delta_u3 = cs.MX.sym('delta_u3')
        delta_u4 = cs.MX.sym('delta_u4')
        delta_u5 = cs.MX.sym('delta_u5')
        delta_u6 = cs.MX.sym('delta_u6')
        self.delta = cs.vertcat(delta_u1, delta_u2, delta_u3, delta_u4, delta_u5, delta_u6)

        # Calculate the rate of change of delta-u
        u_prev = cs.MX.sym('u_prev', 6)
        self.delta = self.u - u_prev

        # Nominal model equations symbolic function (no GP)
        self.quad_xdot_nominal = self.quad_dynamics(rdrv_d_mat)

        # Initialize objective function, 0 target state and integration equations
        self.L = None
        self.target = None

        # Define the position and radius of the obstacle
        self.obs_pos = np.array([5, 4, -2])
        self.obs_radius = 2
        # Build full model. Will have 13 variables. self.dyn_x contains the symbolic variable that
        # should be used to evaluate the dynamics function. It corresponds to self.x if there are no GP's, or
        # self.x_with_gp otherwise
        '''
        acados_models, nominal_with_gp = self.acados_setup_model(
            self.quad_xdot_nominal(x=self.x, u=self.u)['x_dot'], model_name)

        # Convert dynamics variables to functions of the state and input vectors
        self.quad_xdot = {}
        for dyn_model_idx in nominal_with_gp.keys():
            dyn = nominal_with_gp[dyn_model_idx]
            self.quad_xdot[dyn_model_idx] = cs.Function('x_dot', [self.x, self.u], [dyn], ['x', 'u'], ['x_dot'])
        '''
        acados_models, nominal_with_gp = self.acados_setup_model(
            self.quad_xdot_nominal(x=self.x, u=self.u)['x_dot'], model_name)

        # Convert dynamics variables to functions of the state and input vectors
        self.quad_xdot = {}
        for dyn_model_idx in nominal_with_gp.keys():
            dyn = nominal_with_gp[dyn_model_idx]
            self.quad_xdot[dyn_model_idx] = cs.Function('x_dot', [self.x, self.u], [dyn], ['x', 'u'], ['x_dot'])

        # ### Setup and compile Acados OCP solvers ### #
        self.acados_ocp_solver = {}
       
        q_diagonal = np.concatenate((q_cost[:3], np.array([0.1]), q_cost[3:]))
        q_diagonal = np.concatenate((q_diagonal, r_dot_cost))
        

        for key, key_model in zip(acados_models.keys(), acados_models.values()):
            nx = key_model.x.size()[0]
            nu = key_model.u.size()[0]
            ny = nx + nu + nu               # change nu 
            n_param = key_model.p.size()[0] if isinstance(key_model.p, cs.MX) else 0

            # Create OCP object to formulate the optimization
            ocp = AcadosOcp()
            ocp.model = key_model
            ocp.dims.N = self.N
            ocp.solver_options.tf = t_horizon

            # Initialize parameters
            ocp.dims.np = n_param
            ocp.parameter_values = np.zeros(n_param)

            # These two lines of code define the cost function types for the regular stage and the terminal stage, 
            # where cost = 1/2 ∥Vx(x − x_ref) + Vu(u − u_ref)∥_W^2
            # In the terminal stage, the optimizer minimizes the terminal cost function, 
            # coste = 1/2 ∥Vx(x − x_ref)∥_W^2
            ocp.cost.cost_type = 'LINEAR_LS'
            ocp.cost.cost_type_e = 'LINEAR_LS'

            ocp.cost.W = np.diag(np.concatenate((q_diagonal, r_cost)))
            ocp.cost.W_e = np.diag(q_diagonal)
            terminal_cost = 0 if solver_options is None or not solver_options["terminal_cost"] else 1
            ocp.cost.W_e *= terminal_cost

            ocp.cost.Vx = np.zeros((ny, nx))
            ocp.cost.Vx[:nx, :nx] = np.eye(nx)
            ocp.cost.Vu = np.zeros((ny, nu))

            ocp.cost.Vu[nx:nx+nu, :nu] = np.eye(nu)
            ocp.cost.Vu[nx+nu:, :nu] = np.eye(nu) /0.5      # Cost of the rate of change of input

            # ocp.cost.Vx_e = np.eye(nx)
            ocp.cost.Vx_e = np.zeros((len(q_diagonal), nx))
            x_ref = np.zeros(nx + nu)                      
            ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0, 0.0,0.0,0.0])))
            ocp.cost.yref_e = x_ref

            # Initial state (will be overwritten)
            ocp.constraints.x0 = np.zeros(nx)   # Modifiable

            # Set constraints
            ocp.constraints.lbu = np.array([self.min_u] * 6)
            ocp.constraints.ubu = np.array([self.max_u] * 6)
            ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4,5])

            # Solver options, Quadratic Programming 
            ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
            ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
            ocp.solver_options.integrator_type = 'ERK'
            ocp.solver_options.print_level = 0
            ocp.solver_options.nlp_solver_type = 'SQP_RTI' if solver_options is None else solver_options["solver_type"]
            
            # Compile acados OCP solver if necessary
            json_file = os.path.join('./', key_model.name + '_acados_ocp.json')
            self.acados_ocp_solver[key] = AcadosOcpSolver(ocp, json_file=json_file)

    def acados_setup_model(self, nominal, model_name):
        
        """
        Builds an Acados symbolic models using CasADi expressions.
        :param model_name: name for the acados model. Must be different from previously used names or there may be
        problems loading the right model.
        :param nominal: CasADi symbolic nominal model of the quadrotor: f(self.x, self.u) = x_dot, dimensions 13x1.
        :return: Returns a total of three outputs, where m is the number of GP's in the GP ensemble, or 1 if no GP:
            - A dictionary of m AcadosModel of the GP-augmented quadrotor
            - A dictionary of m CasADi symbolic nominal dynamics equations with GP mean value augmentations (if with GP)
        :rtype: dict, dict, cs.MX
        """
        def fill_in_acados_model(x, u, p, dynamics, name):
            x_dot = cs.MX.sym('x_dot', dynamics.shape)
            f_impl = x_dot - dynamics
            # Dynamics model
            model = AcadosModel()
            model.f_expl_expr = dynamics
            model.f_impl_expr = f_impl
            model.x = x
            model.xdot = x_dot
            model.u = u
            model.p = p
            model.name = name

            return model

        acados_models = {}
        dynamics_equations = {}

        # No available GP so return nominal dynamics
        dynamics_equations[0] = nominal
        x_ = self.x
        dynamics_ = nominal
        acados_models[0] = fill_in_acados_model(x=x_, u=self.u, p=[], dynamics=dynamics_, name=model_name)
        return acados_models, dynamics_equations

    def quad_dynamics(self, rdrv_d):
        """
        Symbolic dynamics of the 3D quadrotor model. The state consists on: [p_xyz, a_wxyz, v_xyz, r_xyz]^T, where p
        stands for position, a for angle (in quaternion form), v for velocity and r for body rate. The input of the
        system is: [u_1, u_2, u_3, u_4], i.e. the activation of the four thrusters.

        :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
        by Faessler et al.

        :return: CasADi function that computes the analytical differential state dynamics of the quadrotor model.
        Inputs: 'x' state of quadrotor (6x1) and 'u' control input (2x1). Output: differential state vector 'x_dot'
        (6x1)
        """
        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(rdrv_d), self.w_dynamics())
        u_prev = self.u
        return cs.Function('x_dot', [self.x[:12], self.u], [x_dot], ['x', 'u'], ['x_dot'])

    def p_dynamics(self):
        psi = cs.MX.sym('psi')
        phi = cs.MX.sym('phi')
        theta = cs.MX.sym('theta') 
        phi = self.q[0]; theta = self.q[1]; psi = self.q[2]

        Mat1 = cs.horzcat( cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))
        Mat2 = cs.horzcat( cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi))
        Mat3 = cs.horzcat(         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta))

        Jmat = cs.vertcat(Mat1,Mat2,Mat3) 
        return cs.mtimes(Jmat, self.v)

    def q_dynamics(self):
        psi = cs.MX.sym('psi')
        phi = cs.MX.sym('phi')
        theta = cs.MX.sym('theta') 
        phi = self.q[0]; theta = self.q[1]; psi = self.q[2]
        Mat4 = cs.horzcat(                  1, sin(phi)*tan(theta), cos(phi)*tan(theta))
        Mat5 = cs.horzcat(                  0,            cos(phi),           -sin(phi))
        Mat6 = cs.horzcat(                  0, sin(phi)/cos(theta), cos(phi)/cos(theta))
        Jmat = cs.vertcat(Mat4,Mat5,Mat6) 
        return cs.mtimes(Jmat, self.r)

    def v_dynamics(self, rdrv_d):
        """
        :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
        by Faessler et al. None, if no linear compensation is to be used.
        """
        m     = 14.8     
        W     = 14.8*9.8 - 2
        B     = 14.8*9.8
        Zg    = 0.02
        Xudot = -5.5  # Xu̇
        Yvdot = -12.7 # Yv̇
        Zwdot = -14.57
        Nrdot = -0.12 # Nr˙
        Xu    = -4.03 
        Yv    = -6.22
        Z_w   = -5.18
        
        Nr    = -0.07
        Ixx    = 5.2539 
        Iyy    = 7.9 
        Izz    = 6.9 

        Xuu   = -18.18
        Yvv   = -21.66
        Zww   = -36.99
        Nrr   = -1.55 # Nrr
        v = self.v[1]
        u = self.v[0]
        w = self.v[2]
        p = self.r[0]
        q = self.r[1]
        r = self.r[2]

        K = -cs.vertcat(
        cs.horzcat(0.707, 0.707, -0.707, -0.707, 0, 0),
        cs.horzcat(-0.707, 0.707, -0.707, 0.707, 0, 0),
        cs.horzcat(0, 0, 0, 0, 1, 1),
        cs.horzcat(0.051,  -0.051, 0.051, -0.051, -0.111, 0.111),
        cs.horzcat(0.051,  -0.051, 0.051, -0.051, 0.002, -0.002),
        cs.horzcat(-0.167, 0.167,  0.175, -0.175, 0, 0)
        )

        Force  = cs.mtimes(K,self.u)                                
        X =  Force[0]
        Y =  Force[1]
        Z =  Force[2]
        udot = (1/(m - Xudot))*( X -(m*w - Zwdot*w)*q - (-m*v + Yvdot*v)*r  +  (Xu + Xuu*cs.fabs(u))*u )
        vdot = (1/(m - Yvdot))*( Y - (-m*w + Zwdot*w)*p - (m*u - Xudot*u)*r + (Yv + Yvv*cs.fabs(v))*v )
        wdot = (1/(m - Zwdot))*( Z - (m*v - Yvdot*v)*p - (-m*u + Xudot*u)*q + (Z_w + Zww*cs.fabs(w))*w - W + B ) 
        return cs.vertcat(udot,vdot,wdot)

    def w_dynamics(self):

        m     = 14.8
        W     = 14.8*9.8 - 2
        B     = 14.8*9.8
        Zg    = 0.02
        Xudot = -5.5  # Xu̇
        Yvdot = -12.7 # Yv̇
        Zwdot = -14.57
        Kpdot = -0.12 
        Mqdot = -0.12 
        Nrdot = -0.12 # Nr˙
        Xu    = -4.03 
        Yv    = -6.22       
        Z_w   = -5.18
        
        Nr = -0.07
        Kp = -0.07
        Mq = -0.07

        Ixx    = 5.2539 
        Iyy    = 7.9 
        Izz    = 6.9 
        Xuu   = -18.18
        Yvv   = -21.66
        Zww   = -36.99
        Nrr   = -1.55 # Nrr
        Kpp = -1.55
        Mqq = -1.55

        v = self.v[1]
        u = self.v[0]
        w = self.v[2]
        r = self.r[2]
        p = self.r[0]
        q = self.r[1]
        K = -cs.vertcat(
        cs.horzcat(0.707, 0.707, -0.707, -0.707, 0, 0),
        cs.horzcat(-0.707, 0.707, -0.707, 0.707, 0, 0),
        cs.horzcat(0, 0, 0, 0, 1, 1),
        cs.horzcat(0.051,  -0.051, 0.051, -0.051, -0.111, 0.111),
        cs.horzcat(0.051,  -0.051, 0.051, -0.051, 0.002, -0.002),
        cs.horzcat(-0.167, 0.167,  0.175, -0.175, 0, 0)
        )

        Force  = cs.mtimes(K,self.u) 
        K =  Force[3]
        M =  Force[4]
        N =  Force[5]
  
        pdot = (1/(Ixx - Kpdot))*(K - (m*w - Zwdot*w)*v - (-m*v + Yvdot*v)*w   - (-Iyy*q + Mqdot*q)*r+  (Kp + Kpp*cs.fabs(p))*p)
        qdot = (1/(Iyy - Mqdot))*(M - (-m*w + Zwdot*w)*u - (m*u - Xudot*u)*w  - (Ixx*q - Kpdot*p)*r +  (Mq + Mqq*cs.fabs(q))*q)
        rdot = (1/(Izz - Nrdot))*(N - (m*v - Yvdot*v)*u -(Xudot*u - m*u)*v + (Nr + Nrr*cs.fabs(r))*r )
        return cs.vertcat(pdot,qdot,rdot)


    def run_optimization(self, initial_state=None, goal=None, use_model=0, return_x=False, mode='pose',vel_goal = None):
        """
        Optimizes a trajectory to reach the pre-set target state, starting from the input initial state, that minimizes
        the quadratic cost function and respects the constraints of the system

        :param initial_state: 13-element list of the initial state. If None, 0 state will be used
        :param goal: 3 element [x,y,z] for moving to goal mode, 3*(N+1) for trajectory tracking mode
        :param use_model: integer, select which model to use from the available options.
        :param return_x: bool, whether to also return the optimized sequence of states alongside with the controls.
        :param mode: string, whether to use moving to pose mode or tracking mode
        :return: optimized control input sequence (flattened)
        """

        if initial_state is None:
            initial_state = [0, 0, 0] + [0, 0, 0] + [0, 0, 0] + [0, 0, 0]

        # Set initial state
        x_init = initial_state
        x_init = np.stack(x_init)

        # Set initial condition, equality constraint
        self.acados_ocp_solver[use_model].set(0, 'lbx', x_init)
        self.acados_ocp_solver[use_model].set(0, 'ubx', x_init)

        # Set final condition
        if mode == "pose":
            for j in range(self.N):
                y_ref = np.array([goal[0], goal[1], goal[2], 0,0,0, 0,0,0, 0,0,0, 0,0,0,0,0,0])
                self.acados_ocp_solver[use_model].set(j, 'yref', y_ref)
            y_refN = np.array([goal[0], goal[1], goal[2], 0,0,0, 0,0,0, 0,0,0])
            self.acados_ocp_solver[use_model].set(self.N, 'yref', y_refN)
        else:
            
            for j in range(self.N):
                y_ref = np.array([goal[j,0], goal[j,1], goal[j,2], 0,0,0, vel_goal[j,0],vel_goal[j,1],vel_goal[j,2], 0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0])
                self.acados_ocp_solver[use_model].set(j, 'yref', y_ref)
            y_refN = np.array([goal[self.N,0], goal[self.N,1], goal[self.N,2],0,0,0, vel_goal[j,0],vel_goal[j,1],vel_goal[j,2], 0,0,0, 0,0,0,0,0,0])
            self.acados_ocp_solver[use_model].set(self.N, 'yref', y_refN)
        

        # Solve OCP
        self.acados_ocp_solver[use_model].solve()

        # Get u
        w_opt_acados = np.ndarray((self.N, 6))
        x_opt_acados = np.ndarray((self.N + 1, len(x_init)))
        x_opt_acados[0, :] = self.acados_ocp_solver[use_model].get(0, "x")
        for i in range(self.N):
            w_opt_acados[i, :] = self.acados_ocp_solver[use_model].get(i, "u")
            x_opt_acados[i + 1, :] = self.acados_ocp_solver[use_model].get(i + 1, "x")

        w_opt_acados = np.reshape(w_opt_acados, (-1))
        return w_opt_acados if not return_x else (w_opt_acados, x_opt_acados)