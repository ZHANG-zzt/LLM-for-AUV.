import numpy as np
import math
import timeit
import matplotlib.pyplot as plt
import rclpy
import time
import threading
import json
from mpl_toolkits.mplot3d import Axes3D 
from quadrotor import Quadrotor3D
from controller import Controller
from rclpy.node import Node
from typing import Union, Dict, Optional, Tuple
from geometry_msgs.msg import Pose                # Data format for subscribing to AUV locations
from rclpy.executors import MultiThreadedExecutor # Multi-process training
from std_msgs.msg import String,Float64           # Call data format
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# Published data code added

# Initial parameters to be set
x_speed = None
z_speed = None
y_max = None
period = None

# Setting the added function to accept trajectory coordinates, which is directly added to the original setting function
# --------------------------Setting the added accepted publication weight matrix function---------------------------
class QianfanListener(Node):
    def __init__(self):
        super().__init__('parameter_listener')
        # Subscribe to the parameters from the LLM output.
        self.subscription = self.create_subscription(
            String,
            '/bluerov2/parameter_topic',
            self.listener_callback,
            10)
        # Subscribe to the trajectories from the LLM outputs.
        self.trajectory_subscription = self.create_subscription(
            String,
            '/bluerov2/trajectory_topic',
            self.trajectory_callback,
            10)
        self.subscription              # prevent unused variable warning
        self.trajectory_subscription   # prevent unused variable warning 
    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        # Extract specific fields of track instructions
        try:
            data = json.loads(msg.data)
            weight_matrix = data.get('weight_matrix', {}).get('value', None)
            if weight_matrix:
                self.get_logger().info(f'Weight matrix: {weight_matrix}')
                # Handle the values of the weight_matrix, such as publishing other messages or performing additional actions.
            else:
                self.get_logger().warn('No weight matrix found in the received message.')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {str(e)}')
    
    def trajectory_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')
        try:
            trajectory_value = json.loads(msg.data)
            self.get_logger().info(f'Weight matrix: {trajectory_value}')
            if trajectory_value is not None and isinstance(trajectory_value, dict):
                global x_speed, z_speed, y_max, period
                x_speed = trajectory_value.get('X_speed', None)
                z_speed = trajectory_value.get('Z_speed', None)
                y_max = trajectory_value.get('y_max', None)
                period = trajectory_value.get('period', None)
                
                # Output of command information through the logging function
                self.get_logger().info(f'X_speed: {x_speed}')
                self.get_logger().info(f'Z_speed: {z_speed}')
                self.get_logger().info(f'y_max: {y_max}')
                self.get_logger().info(f'period: {period}')
            else:
                self.get_logger().warning('Invalid format for trajectory_value.')
        # Handling of Command Message Errors 
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')
    

#----------------------------------Real-time display of AUV trajectories in Rviz2-------------------------------
class RobotTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('robot_trajectory_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.marker = Marker()
        self.marker.header.frame_id = 'world'  # Setting the frame_id to 'world'
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.scale.x = 0.05  # Line width
        self.marker.color.a = 1.0   # Transparency
        self.marker.color.r = 0.0   # Red component
        self.marker.color.g = 1.0   # Green component
        self.marker.color.b = 0.0   # Blue component

    def publish_trajectory(self, position):
        point = Point()
        point.x = position[0]
        point.y = position[1]
        point.z = position[2]

        self.marker.points.append(point)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker)

# ----------------------Setting a defined curve trajectory------------------------------------
class LinePublisher(Node):
    def __init__(self):
        super().__init__('line_publisher')
        self.line_publisher_ = self.create_publisher(Marker, '/line', 10)
        
        # Default initial parameters, if a user command is received within a certain period of time, tracking by default parameters.
        self.slope = 0.7
        self.amplitude = 50
        self.frequency = 0.15
        self.offset = 0.1
        self.timer = self.create_timer(0.1, self.publish_line)

    def publish_line(self):

        global x_speed, z_speed, y_max, period
        # If a message is received from the LLM, run the AUV according to the new instructions
        if x_speed is not None and z_speed is not None and y_max is not None and period is not None:
            self.slope = float(x_speed)
            self.amplitude = float(10*y_max)
            self.offset = z_speed
        else:
            None
        # Define the basic parameters of the Rviz2 display trajectory
        line_msg = Marker()
        line_msg.header.frame_id = 'world'
        line_msg.header.stamp = self.get_clock().now().to_msg()
        line_msg.type = Marker.LINE_STRIP
        line_msg.action = Marker.ADD
        line_msg.pose.orientation.w = 1.0
        line_msg.scale.x = 0.05                     # Line width
        line_msg.color.r = 1.0
        line_msg.color.a = 1.0
        
        # Define the line points that show the desired trajectory
        dt = 0.1                                     
        for i in range(800):
            point = Point()
            point.x =  -self.slope*dt*i + 5
            point.y =  self.amplitude*dt*math.sin(self.frequency*dt*i)
            point.z =  -1.2 - self.offset*dt*i
            line_msg.points.append(point)
        self.line_publisher_.publish(line_msg)

    
#-----------------------Set the node message that publishes the thruster speed-------------------------------
class Bluerov2_Trust(Node):
    def __init__(self):
      super().__init__('bluerov_truster')
      self.truster1_publisher = self.create_publisher(Float64, '/bluerov2/cmd_thruster1', 1)
      self.truster2_publisher = self.create_publisher(Float64, '/bluerov2/cmd_thruster2', 1)
      self.truster3_publisher = self.create_publisher(Float64, '/bluerov2/cmd_thruster3', 1)
      self.truster4_publisher = self.create_publisher(Float64, '/bluerov2/cmd_thruster4', 1)
      self.truster5_publisher = self.create_publisher(Float64, '/bluerov2/cmd_thruster5', 1)
      self.truster6_publisher = self.create_publisher(Float64, '/bluerov2/cmd_thruster6', 1)


# Storage of thruster data
thrust_1 = []
thrust_2 = []
thrust_3 = []
thrust_4 = []
thrust_5 = []
thrust_6 = []

# ---------------------------------Reference Path-------------------------------------
def createTrajectory(sim_time, dt):
    xref = []; yref = []; zref = []
    vel_x = 0.7
    sway_max = 50
    vel_z = 0.1
    global x_speed, z_speed, y_max, period
    if x_speed is not None and z_speed is not None and y_max is not None and period is not None:
            # self.get_logger().info("x_speed, z_speed, y_max, and period are all not None")
            vel_x = float(x_speed)
            sway_max = float(10*y_max)
            vel_z  = z_speed
    else:
            None
    for i in range(int(sim_time/(dt))):
        x =  -vel_x*dt*i + 5
        y =  sway_max*dt*math.sin(0.15*dt*i)
        z = - 0.2 - vel_z*dt*i
        xref.append(x)
        yref.append(y)
        zref.append(z)
    return np.array(xref), np.array(yref), np.array(zref)

# ------------------------Reference Velocity--------------------------------
def calculateVelocity(xref, yref, zref, dt):
    # Calculate the differences between consecutive points
    dx = np.diff(xref)
    dy = np.diff(yref)
    dz = np.diff(zref)
    
    # Calculate the velocities
    vx = dx / dt
    vy = dy / dt
    vz = dz / dt
    
    # Since diff reduces the array length by 1, we add a zero at the end
    # or use the last velocity value to keep the array length consistent
    vx = np.append(vx, vx[-1])
    vy = np.append(vy, vy[-1])
    vz = np.append(vz, vz[-1])   
    return vx, vy, vz


def trackTrajectory():
    dt = 0.1                                 # Time step, sampling Interval 
    N = 20                                   # NMPC Prediction Horizon
    sim_time = 70                            # Total simulation time defined
    global x_speed, z_speed, y_max, period

    rclpy.init()
    quad = Quadrotor3D()                     # Quadrotor model
    controller = Controller(quad, t_horizon= 2*N*dt, n_nodes=N)  # Initialize NMPC controller
    
    # Initialize each function
    node_trust = Bluerov2_Trust()
    line_publisher = LinePublisher()
    robot_trajectory_publisher = RobotTrajectoryPublisher()
    executor = MultiThreadedExecutor() 
    Qianfan  = QianfanListener()             # Introducing nodes from LLM.    
   
    # Start each ROS2 node
    executor.add_node(node_trust)
    executor.add_node(line_publisher)
    executor.add_node(quad.Gazebo_VelState)
    executor.add_node(quad.node_path)
    executor.add_node(robot_trajectory_publisher)
    executor.add_node(Qianfan)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Get the timestamp of the current time。Set the wait time, 
    # if it is greater than 5 seconds, follow the default parameters for tracking tasks
    start_time = time.time()                 
    max_wait_time = 5
    while time.time() - start_time < max_wait_time:
        if x_speed is not None and z_speed is not None and y_max is not None and period is not None:
            xref, yref, zref = createTrajectory(sim_time, dt)    # Reference path
            break                                                # If all parameters are set, the loop is skipped
        time.sleep(1)                                            # Checked every second

    xref, yref, zref = createTrajectory(sim_time, dt)            # Reference path
    vx, vy, vz = calculateVelocity(xref, yref, zref, dt)         # Reference velcity

    path = []
    VEL  = []
    ATTITUDE =[]
    # Tasks related to the main loop
    time_record = []
    for i in range(int(sim_time/dt)):
        
        x = xref[i:i+N+1]; y = yref[i:i+N+1]; z = zref[i:i+N+1]
        v_x = vx[i: i+N+1] ; v_y = vy[i: i+N+1] ; v_z = vz[i: i+N+1]
        if len(x) < N+1:
            x = np.concatenate((x,np.ones(N+1-len(x))*xref[-1]),axis=None)
            y = np.concatenate((y,np.ones(N+1-len(y))*yref[-1]),axis=None)
            z = np.concatenate((z,np.ones(N+1-len(z))*zref[-1]),axis=None)
            v_x = np.concatenate((v_x,np.ones(N+1-len(v_x))*vx[-1]),axis=None)
            v_y = np.concatenate((v_y,np.ones(N+1-len(v_y))*vy[-1]),axis=None)
            v_z = np.concatenate((v_z,np.ones(N+1-len(v_z))*vz[-1]),axis=None)
        goal = np.array([x,y,z]).T
        vel_ref = np.array([v_x,v_y,v_z]).T

        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        start = timeit.default_timer()
        thrust = controller.run_optimization(initial_state=current, goal=goal, mode='traj',vel_goal = vel_ref)[:6]
        thrust_1.append(50*thrust[0])
        thrust_2.append(50*thrust[1])
        thrust_3.append(50*thrust[2])
        thrust_4.append(50*thrust[3])
        thrust_5.append(50*thrust[4])
        thrust_6.append(50*thrust[5])

#----------------------------------Release of data on propellers for AUVs---------------------------
        trust_list = np.zeros(6) 
        for i in range(6):                                # Distribution of 6 thrusters
            thruster_force = Float64() 
            if i == 4 or 5:
                thruster_force.data = float(-50*thrust[i])
            else:    
                thruster_force.data = float(50*thrust[i])
            getattr(node_trust, f'truster{i+1}_publisher').publish(thruster_force)

#--- Wait 0.1s, the same sampling interval, and then collect the new status of the AUV from the gazebo environment--------
        time.sleep(0.1)
        time_record.append(timeit.default_timer() - start)
        quad.update(thrust, dt)       # AUV Thruster status update
        path.append(quad.pos)         # Saving of AUV position information
        robot_trajectory_publisher.publish_trajectory(quad.pos)
        VEL.append(quad.vel)          # AUV Speed information saved 
        ATTITUDE.append(quad.angle)   # Save attitude information


    # Calculate how long it takes the CPU to compute the optimization problem at each control step
    print("average estimation time is {:.5f}".format(np.array(time_record).mean()))
    print("max estimation time is {:.5f}".format(np.array(time_record).max()))
    print("min estimation time is {:.5f}".format(np.array(time_record).min()))

    # -----------------------------Show 3D motion trajectory in figure ------------------
    path = np.array(path)
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(xref, yref, zref, c=[1,0,0], label='Goal from User Set')
    ax.plot(path[:,0], path[:,1], path[:,2], label='Bluerov2 Path from NMPC')
    ax.axis('auto')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    # ax.set_zlim(-1, 0)                
    ax.legend()

    # -------------------------Display the error of trajectory tracking----------------------------------
    plt.figure()
    bx = plt.axes(projection='3d')
    bx.plot(xref - path[:,0], yref - path[:,1], zref - path[:,2], label='track error')
    bx.axis('auto')
    bx.set_xlabel('x [m]')
    bx.set_ylabel('y [m]')
    bx.set_zlabel('z [m]')
    bx.legend()

    #---------------------------Calculate the square root error of trajectory tracking-----------------
    error = np.sqrt((xref - path[:,0])**2 + (yref - path[:,1])**2 + (zref - path[:,2])**2)
    plt.figure()
    plt.plot((error),label='track error')
    plt.ylabel('track error')
    plt.xlabel('time')

    # ---------------------------------Displays the time of GPU calculations ------------------
    plt.figure()
    plt.plot(time_record)
    plt.ylabel('CPU Time [s]')

    # --------------------------Forms for calculating total energy consumption-----------
    total_thrust = []
    total_thrust = np.abs(thrust_1)+ np.abs(thrust_2) + np.abs(thrust_3)+\
                    np.abs(thrust_4)+ np.abs(thrust_5) + np.abs(thrust_6)
    # np.savetxt('result/total_thrust_0.1.txt', total_thrust)

    # plt.figure()
    # plt.plot((thrust_1),label='Trust_FL')
    # plt.plot((thrust_2),label='Trust_FR')
    # plt.plot(thrust_3,label='Trust_BL')
    # plt.plot(thrust_4,label='Trust_BR')
    # plt.plot(thrust_5,label='Trust_UL')
    # plt.plot(thrust_6,label='Trust_UR')
    # plt.ylabel('Trust_Force')
    # plt.xlabel('time')
    # plt.legend()

 #------------------------------------- AUV motion speed during trajectory tracking+----------------------------------
    VEL = np.array(VEL)
    plt.figure()
    plt.plot(VEL[:,0], label='u', linestyle='--', color='Blue', linewidth=2)
    plt.plot(VEL[:,1], label='v', linestyle='--', color='red',linewidth=2)
    plt.plot(VEL[:,2], label='w', linestyle='--', color='green',linewidth=2)
    plt.ylabel('Velocity($m/s$)',fontsize=18)
    plt.xlabel('Time ($0.1s$)',fontsize=18)
    plt.legend(loc='upper right', fontsize=14)
    plt.grid(True)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    
#----------------------------Displays attitude information during trajectory tracking--------------------------------------
    ATTITUDE = np.array(ATTITUDE)
    plt.figure()
    plt.plot(ATTITUDE[:,0], label='phi', linestyle='--', color='Blue',linewidth=2)
    plt.plot(ATTITUDE[:,1], label='theta', linestyle='--', color='red',linewidth=2)
    plt.plot(ATTITUDE[:,2], label='psi', linestyle='--', color='green',linewidth=2)
    plt.ylabel('Attitude',fontsize=18)
    plt.xlabel('Time ($0.1s$)',fontsize=18)
    plt.legend(loc='upper right', fontsize=14)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    plt.ylim(-2.5,1)
    plt.grid(True)

    # plt.figure()
    # plt.plot(vx, label='velocity-x')
    # plt.plot(vy, label='velocity-y')
    # plt.plot(vz, label='velocity-z')
    # plt.ylabel('Reference velocity')
    # plt.xlabel('time')
    # plt.legend()
    # plt.show()

if __name__ == "__main__":
    trackTrajectory()