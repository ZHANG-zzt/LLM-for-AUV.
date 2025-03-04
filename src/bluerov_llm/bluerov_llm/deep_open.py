import time 
import json
from openai import OpenAI
import os

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry
import argparse
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor

client = OpenAI(
    api_key="Your-api_key",
    base_url="https://api.deepseek.com",  
)
def set_input_matrix(array):
    return {
        "action": "set_input_matrix",
        "params": {
            "weight_matrix": {
                "type": "array",
                "value": array
            }
        }
    }
def set_nmpc_matrix(array):
    return {
        "action": "change_mpc_weight",
        "params": {
            "weight_matrix": {
                "type": "array",
                "value": array
            }
         }
    }
def move_point(x, y, z):
    return {
        "action": "move_to_point",
        "params": {
            "x": x,
            "y": y,
            "z": z
        }
    }
def Move_bluerov2_speed(linear_speed,current_speed):
    return {
        "action": "move_bluerov2",
        "params": {
            "linear_speed": linear_speed,
            "current_speed": current_speed,
        }
    }

def set_trajectory(X_speed, Z_speed, Y_max, period):
    return {
        "action": "set_trajectory",
        "params": {
            "X_speed": X_speed,
            "Z_speed": Z_speed,
            "Y_max": Y_max,
            "period": period
        }
    }

system_prompt = """
        Consider the following ontology:
        {"action": "go_to_goal", "params": {"location": {"type": "str", "value": location}}}
        {"action": "move", "params": {"linear_speed": linear_speed, "distance": distance, "is_forward": is_forward}}
        {"action": "rotate", "params": {"angular_velocity": angular_velocity, "angle": angle, "is_clockwise": is_clockwise}}
        {"action": "move_to_point", "params": {"x": x, "y": y, "z": z}}
        {"action": "move_bluerov2", "params": {"speed_change": speed_change}}
        rules:  If the user's instruction is about setting the AUV state variable weights.
        def set_input_matrix(array):
            return {
                "action": "set_input_matrix",
                "params": {
                    "weight_matrix": {
                        "type": "array",
                        "value": value}}}
        def set_nmpc_matrix(array):
            return {
                "action": "change_mpc_weight",
                "params": {
                    "weight_matrix": {
                        "type": "array",
                        "value": value}}}
        {
        "action": "set_trajectory",
        "params": {
            "X_speed": X_speed,
            "Z_speed": Z_speed,
            "y_max": y_max,
            "period": period
            }
        }
        You will be given human language prompts, and you need to return a JSON conformant to the ontology. Any action not in the ontology must be ignored. Here are some examples.
        prompt: "Move the bluerov2 faster 0.2, current speed is 0.3 m/s."
        returns: Move_bluerov2_speed(0.2, 0.3)

        prompt: "Move forward for 1 meter at a speed of 0.5 meters per second."
        returns: {"action": "move", "params": {"linear_speed": 0.5, "distance": 1, "is_forward": true, "unit": "meter"}

        prompt: "Move the Bluerov2 to the point x=10.0, y=10.0, z=-3.0."
        returns: move_point(x=10.0, y=10.0, z=-3.0)

        prompt: "Change the NMPC weight matrix to [20,20,20,0.1,0.1,0.1]."
        returns: set_nmpc_matrix([20,20,20,0.1,0.1,0.1])

        # prompt: "set the control input weight matrix to diag(0.2, 0.2, 0.2, 0.2, 0.2, 0.2)."
        # returns: set_input_matrix(diag(0.2, 0.2, 0.2, 0.2, 0.2, 0.2))

        prompt: "Move the bluerov2 slower by 0.2 m/s"
        returns: 
        {
            "action": "move_bluerov2",
            "params": 
                {
                    "speed_change": -0.2,
                }
            "ros2_subscribe":
                {
                    "topic": "/bluerov2/odom1",
                    "message_type": "nav_msgs/msg/Odometry", 
                    "callback_function": "odom_callback"
                }
        }

        prompt: "Navigate the bluerov2 to locate directly below the obstacle, the obstacle direction is (x=10.0, y=20.0,z =-10.0)."
        returns: move_point(the obstacle direction)

        # prompt: "Set the trajectory with the speeds along the X and Z axes set to 1.0 m/s and 0.5 m/s respectively, while the Y-coordinate follows a sine wave with a period of 15 seconds and an amplitude of 10 meters." 
        # return: set_trajectory(X_speed = 1.0, Z_speed = -0.5, Y_max =10, period = 15)
"""

def generate_command_prompt(user_command):
    return f"""
Current Task: Convert the following natural language commands to standard JSON control commands
Instruction: {user_command}
"""

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber_node')
        
        self.subscription = self.create_subscription(
            Pose,
            '/bluerov2/pose_gt',
            self.listener_callback,
            10)
        self.subscription                                     # Prevent unused variable warning
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/bluerov2/odom1',
            self.odom_callback,
            1)
        self.speed_buffer = []  
        self.max_buffer_size = 10
    def listener_callback(self, msg): 
        return
    
    def set_speed_reduction(self, speed_reduction):
        self.speed_reduction = speed_reduction
   
    def odom_callback(self, msg: Odometry):
        global average_speed
        current_speed = msg.twist.twist.linear.x
        self.speed_buffer.append(current_speed)
        # Process the average value to ensure the transmission and reception frequencies are consistent.
        if len(self.speed_buffer) == self.max_buffer_size:                
            average_speed = sum(self.speed_buffer) / len(self.speed_buffer)
            self.get_logger().info(f'current_speed: {average_speed}')
            self.speed_buffer.clear()

class QianfanCommandPublisher(Node):
    def __init__(self):
        super().__init__('qianfan_command_publisher')
        self.publisher_1 = self.create_publisher(PoseStamped, '/bluerov2/cmd_pose', 10)     #publish the position.
        self.publisher_2 = self.create_publisher(String, '/bluerov2/parameter_topic', 10)   #publish the parameter.
        self.publisher_3 = self.create_publisher(String, '/bluerov2/trajectory_topic', 10)  #publish the trajectory.
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.current_command = None
        self.get_logger().info('Qianfan Command Publisher has been started.')
        
    def listener_callback(self, msg):
        self.pose_zz = msg
    
    def parse_control_command(self, command):
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": generate_command_prompt(command)}
        ]
        self.get_logger().info(f"Received action")
        try:
            start_time = time.time()  # Write start  time
            response = client.chat.completions.create(
                model="deepseek-chat",
                messages=messages,
                temperature=0.3,  # Reduce randomness
                top_p=0.95,
                max_tokens=128,
                response_format={"type": "json_object"},
                timeout=15.0        # Timeout control
            )

            # response = client.chat.completions.create(
            #     model = "deepseek-reasoner",
            #     messages=messages,
            # )
            end_time = time.time()    # Write end time
            self.get_logger().info(f"response recieved")
            elapsed_time = end_time - start_time  
            self.get_logger().info(f"Reasoning Time: {elapsed_time:.4f} s")
            response_content = response.choices[0].message.content
            json_str = response_content.replace("```json", "").replace("```", "").strip()

            # Parse JSON
            parsed_json = json.loads(json_str)
            self.current_command = parsed_json
            # Extract key values
            action = parsed_json.get('action', '')
            params = parsed_json.get('params', {})
            weight_matrix = params.get('weight_matrix', {}).get('value')

            # Use ROS 2 logging instead of print()
            self.get_logger().info(f"Received action: {action}")
            self.get_logger().info(f"Received params: {params}")

            return parsed_json  # Return the parsed JSON object

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"API request failed: {str(e)}")
            return None

    def timer_callback(self):
        
        if self.current_command and self.current_command.get('action') == 'set_input_matrix':
            input_matrix = self.current_command.get('params', {})
            self.get_logger().info(f"input_matrix value: {input_matrix}")

        if self.current_command and self.current_command.get('action') == 'set_trajectory':
            msg = String()
            trajectory = self.current_command.get('params', {})
            msg.data = json.dumps(trajectory)
            self.get_logger().info(f"trajectory value: {msg.data}")
            self.publisher_3.publish(msg)
            # self.get_logger().info(f"trajectory value: {msg}")
        
        if self.current_command and self.current_command.get('action') == 'move_bluerov2':
            global average_speed
            move_bluerov2 = self.current_command.get('params', {}).get('speed_change', {})
            desired_speed = float(move_bluerov2) + average_speed
            self.get_logger().info(f"Blurov2 desired_speed: {desired_speed}")

        if self.current_command and self.current_command.get('action') == 'move_to_point':
            params = self.current_command.get('params', {})
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = params.get('x', 0.0)
            pose_msg.pose.position.y = params.get('y', 0.0)
            pose_msg.pose.position.z = params.get('z', 0.0)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            self.publisher_1.publish(pose_msg)
            self.get_logger().info(f"Published Pose: {pose_msg}")
        if self.current_command and self.current_command.get('action') == 'change_mpc_weight':
            # self.get_logger().info(f"Published parameters: {self.current_command.get('params')}")
            weight_matrix = self.current_command.get('params', {}).get('weight_matrix', {}).get('value')
            if weight_matrix:
                self.get_logger().info(f"Published Pose:")
                weight_msg = String()
                weight_msg.data = json.dumps({'weight_matrix': {'type': 'array', 'value': weight_matrix}})
                self.publisher_2.publish(weight_msg)
                self.get_logger().info(f"Published Pose: {weight_msg}")
            else:
                self.get_logger().warn('No weight matrix found in the received message.')

def main(args=None):
    rclpy.init(args=args)
    node_1 = QianfanCommandPublisher()
    node_2 = PoseSubscriber()
    
    parser = argparse.ArgumentParser(description="Convert natural language commands to JSON and publish as ROS2 message using Qianfan.")
    parser.add_argument('command', type=str, help='The natural language command to convert and publish.')

    args = parser.parse_args()
    node_1.parse_control_command(args.command)

    executor = MultiThreadedExecutor()   # Create a ROS2 multithreaded actuator
    executor.add_node(node_1)
    executor.add_node(node_2)

    try:
        executor.spin()                  # Execution node
    finally:
        node_1.destroy_node()            # Destroy nodes and turn off ROS 2
        node_2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

