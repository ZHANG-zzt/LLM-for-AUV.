from math import sqrt
import numpy as np
from utils import quaternion_to_euler, skew_symmetric, v_dot_q, unit_quat, quaternion_inverse
import geomutils as geom
import state_space_3d as ss
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import glob
import os
from typing import Union, Dict, Optional, Tuple
from geometry_msgs.msg import Pose, Point        
from rclpy.executors import MultiThreadedExecutor 
from std_msgs.msg import String,Float64   
from nav_msgs.msg import Odometry  
import tf_transformations
import time
import threading
from visualization_msgs.msg import Marker

#------------------------Implement the display of the motion trajectory.------------------------------
class RobotTrajectoryPublisher(Node):
    def __init__(self, quad_pos):
        super().__init__('robot_trajectory_publisher')
        self.trajectory_publisher_ = self.create_publisher(Marker, '/robot_trajectory', 10)
        self.timer = self.create_timer(0.1, self.publish_trajectory)
        self.quad_pos = quad_pos
        self.current_position = Point()

    def read_latest_position(self):
        if self.quad_pos:
            # Assuming quad_pos is a list containing the latest position information
            latest_position = self.quad_pos[-1]
            self.current_position.x = latest_position[0]
            self.current_position.y = latest_position[1]
            self.current_position.z = latest_position[2]
    # Trajectory generation function
    def publish_trajectory(self):
        trajectory_msg = Marker()
        trajectory_msg.header.frame_id = 'world'
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.type = Marker.LINE_STRIP
        trajectory_msg.action = Marker.ADD
        trajectory_msg.pose.orientation.w = 1.0
        trajectory_msg.scale.x = 0.05  # Line width
        trajectory_msg.color.g = 1.0   # Green color
        trajectory_msg.color.a = 1.0   # Full opacity
        self.read_latest_position()
        trajectory_msg.points.append(self.current_position)
        self.trajectory_publisher_.publish(trajectory_msg)

#------------------------Create a subscriber for the path messages.------------------------------     
class Bluerov2_Path(Node):
    def __init__(self):
        super().__init__('bluerov_path')
        self.path_sub = self.create_subscription(Pose, '/bluerov2/pose_gt', self.topic_callback,1)
        self.path_msg = None
        self.path_sub                                 # Prevent the subscriber from being garbage collected (GC).
    def topic_callback(self, msg):
        self.path_msg = msg  
    def get_message(self):
        return self.path_msg

class Gazebo_Subscriber(Node):
    def __init__(self):
        super().__init__('Gazebo_Subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/bluerov2/odom1',  
            self.topic_callback,
            1)
        self.last_msg = None
        self.subscription
    def topic_callback(self, msg):
        self.last_msg = msg
    def get_message(self):
       return self.last_msg

# ------------------------------The state equations set for the AUV's 6-DOF.------------------------------
dt = 0.1                                  # The set minimum sampling time.
def state_dot(state,trust):

    state = np.array(state)
    eta =  np.hstack([state[0],state[1]])
    eta = eta.T
    nu_r = np.hstack([state[2],state[3]])
    nu_r = nu_r.T
    eta_dot = geom.J(eta).dot(nu_r)
    nu_r_dot = ss.M_inv().dot(
        ss.B(nu_r).dot(trust)
        - ss.D(nu_r).dot(nu_r)
        - ss.C(nu_r).dot(nu_r)
        - ss.G(eta)
        )
    state_dot = np.hstack([eta_dot, nu_r_dot])
    return state_dot

# ------------------------------The state equations set for the AUV's 6-DOF.------------------------------
def Second_step(state,trust):
    eta =  state[:6]
    nu_r = state[6:]
    eta_dot = geom.J(eta).dot(nu_r)
    nu_r_dot = ss.M_inv().dot(
        ss.B(nu_r).dot(trust)
        - ss.D(nu_r).dot(nu_r)
        - ss.C(nu_r).dot(nu_r)
        - ss.G(eta)
        )
    state_dot = np.hstack([eta_dot, nu_r_dot])
    # print('state_dot',state_dot*dt + np.hstack([eta.T, nu_r.T]))
    return state_dot

# --------------------------------Fourth-order Runge-Kutta equations, used for the prediction model.------------------------------
def rk4_step(state, trust, dt):

    state = np.array(state)
    eta =  np.hstack([state[0],state[1]])
    eta = eta.T
    nu_r = np.hstack([state[2],state[3]])
    nu_r = nu_r.T
    state= np.hstack([eta ,nu_r])

    k1 = Second_step(state, trust)
    k2 = Second_step(state + 0.5 * dt * k1, trust)
    k3 = Second_step(state + 0.5 * dt * k2, trust)
    k4 = Second_step(state + dt * k3, trust)
    new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return new_state

# --------------------------The set heading constraint equation: 0 to 2π.------------------------------
def wrap(aic_psi):
    while aic_psi > np.pi:
        aic_psi -= 2*np.pi
    while aic_psi < -np.pi:
        aic_psi += 2*np.pi
    return aic_psi 

class Quadrotor3D:

    def __init__(self, noisy=False, drag=False, payload=False, motor_noise=False):
        """
        Initialization of the 3D quadrotor class
        :param noisy: Whether noise is used in the simulation
        :type noisy: bool
        :param drag: Whether to simulate drag or not.
        :type drag: bool
        :param payload: Whether to simulate a payload force in the simulation
        :type payload: bool
        :param motor_noise: Whether non-gaussian noise is considered in the motor inputs
        :type motor_noise: bool
        """
        self.Gazebo_VelState = Gazebo_Subscriber() 
        self.node_path  = Bluerov2_Path()
        # self.plot_path  = RobotTrajectoryPublisher()

        # Maximum thrust in Newtons of a thruster when rotating at maximum speed.
        self.max_thrust = 50
        # System state space
        self.pos = np.zeros((3,))
        self.vel = np.zeros((3,))
        self.angle = np.array([0., 0., 0.])  # Quaternion format: qw, qx, qy, qz
        self.a_rate = np.zeros((3,))

        # Input constraints
        self.max_input_value = 1     # Maximum thrust for forward propulsion rotation.
        self.min_input_value = -0.7  # Maximum thrust for reverse propulsion rotation.

        # Actuation thrusts
        self.u_noiseless = np.array([0.00, 0.00, 0.00, 0.0,0.0,0.0])
        self.u = np.array([0.0, 0.0, 0.0, 0.0,0.0,0.0])                        # N

        self.drag = drag
        self.noisy = noisy
        self.motor_noise = motor_noise

    def set_state(self, *args, **kwargs):
        if len(args) != 0:
            assert len(args) == 1 and len(args[0]) == 13
            self.pos[0], self.pos[1], self.pos[2], \
            self.angle[0], self.angle[1], self.angle[2], \
            self.vel[0], self.vel[1], self.vel[2], \
            self.a_rate[0], self.a_rate[1], self.a_rate[2] \
                = args[0]
        else:
            self.pos = kwargs["pos"]
            self.angle = kwargs["angle"]
            self.vel = kwargs["vel"]
            self.a_rate = kwargs["rate"]

    def get_state(self, quaternion=False, stacked=False):

        if quaternion and not stacked:
            return [self.pos, self.angle, self.vel, self.a_rate]
        if quaternion and stacked:
            return [self.pos[0], self.pos[1], self.pos[2], self.angle[0], self.angle[1], self.angle[2],
                    self.vel[0], self.vel[1], self.vel[2], self.a_rate[0], self.a_rate[1], self.a_rate[2]]

        angle = quaternion_to_euler(self.angle)
        if not quaternion and stacked:
            return [self.pos[0], self.pos[1], self.pos[2], angle[0], angle[1], angle[2],
                    self.vel[0], self.vel[1], self.vel[2], self.a_rate[0], self.a_rate[1], self.a_rate[2]]
        return [self.pos, angle, self.vel, self.a_rate]

    def get_control(self, noisy=False):
        if not noisy:
            return self.u_noiseless
        else:
            return self.u

    def update(self, u, dt):
        """
        Runge-Kutta 4th order dynamics integration

        :param u: 4-dimensional vector with components between [0.0, 1.0] that represent the activation of each motor.
        :param dt: time differential
        """
        # Clip inputs
        for i, u_i in enumerate(u):
            self.u_noiseless[i] = max(min(u_i, self.max_input_value), self.min_input_value)

        # Apply noise to inputs (uniformly distributed noise with standard deviation proportional to input magnitude)
        if self.motor_noise:
            for i, u_i in enumerate(self.u_noiseless):
                std = 0.02 * sqrt(u_i)
                noise_u = np.random.normal(loc=0.1 * (u_i / 1.3) ** 2, scale=std)
                self.u[i] = max(min(u_i - noise_u, self.max_input_value), self.min_input_value) * self.max_thrust
        else:
            self.u = self.u_noiseless * self.max_thrust

        # Generate disturbance forces / torques
        if self.noisy:
            f_d = np.random.normal(size=(3, 1), scale=10 * dt)
            t_d = np.random.normal(size=(3, 1), scale=10 * dt)
        else:
            f_d = np.zeros((3, 1))
            t_d = np.zeros((3, 1))

        x = self.get_state(quaternion=True, stacked=False)
        x = rk4_step(x,self.u,dt)

        #--------------------------Subscription methods in the platform -----------------------------
        last_message = self.Gazebo_VelState.get_message()
        if last_message is not None:
            linear_velocity = last_message.twist.twist.linear             # Linear velocity
            angular_velocity = last_message.twist.twist.angular           # Angular_velocity
            orientation = last_message.pose.pose.orientation              # Rotation, four sources    
        else:
            linear_velocity = None
            orientation = None
        # ------------------------Linear velocity for handling AUVs ----------------------------
        if linear_velocity is None:
            Vel_x = 0
            Vel_y = 0
            Vel_z = 0
        else:
            Vel_x = linear_velocity.x
            Vel_y = linear_velocity.y
            Vel_z = linear_velocity.z
        # ------------------------Handling the angular velocity of the AUV---------------------------
        if angular_velocity is None:
            angular_p = 0
            angular_q = 0
            angular_r = 0
        else:
            angular_p = angular_velocity.x
            angular_q = angular_velocity.y
            angular_r = angular_velocity.z
        # -----------------------Handling the positional status of AUVs---------------------------
        if orientation is None:
            qx = 0
            qy = 0
            qz = 0
            qw = 0
        else:
            qx = orientation.x
            qy = orientation.y
            qz = orientation.z
            qw = orientation.w
            auv_pose = self.node_path.get_message()
        # -----------------------Processing of AUV position information---------------------------    
        if auv_pose is not None:     
            aic_x = auv_pose.position.x
            aic_y = auv_pose.position.y
            aic_z = auv_pose.position.z
            self.new_data_received = True
        else:
            aic_x = 0
            aic_y = 0 
            aic_z = 0

        #Converts quaternions (representing three-dimensional rotations) to Euler angles (roll, pitch, and yaw).
        orientation_new = [qx,qy,qz,qw]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_new)
        self.pos      =   [aic_x,aic_y,aic_z]
        self.angle    =   [roll, pitch, yaw]
        self.angle[2] = wrap(self.angle[2])
        self.vel      =   [Vel_x, Vel_y, Vel_z]
        self.a_rate   =   [angular_p,angular_q,angular_r]




