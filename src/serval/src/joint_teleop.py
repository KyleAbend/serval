#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JointTeleopNode(Node):
    def __init__(self):
        super().__init__('joint_teleop_node')
        
        # Configuration
        self.declare_parameter('joystick_axis_0', 0)        # Left stick X for joint_0
        self.declare_parameter('joystick_axis_1', 1)        # Left stick Y for joint_1
        self.declare_parameter('joint_speed_scale', 1)
        self.declare_parameter('max_joint_position', 1.5707963)  # ~90 degrees
        self.declare_parameter('min_joint_position', -1.5707963) # ~-90 degrees
        self.axis_0 = self.get_parameter('joystick_axis_0').value
        self.axis_1 = self.get_parameter('joystick_axis_1').value
        self.speed_scale = self.get_parameter('joint_speed_scale').value
        self.max_pos = self.get_parameter('max_joint_position').value
        self.min_pos = self.get_parameter('min_joint_position').value
        
        # Track current positions for all joints (12 joints total)
        self.joint_positions = [0.0] * 12  # FR, FL, BR, BL (3 per leg)
        
        # Publisher for joint commands (read by joint_states_publisher)
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )
        
        # Subscriber to joystick input
        self.sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        self.get_logger().info('Joint teleoperation node started. PS3 controller ready.')
        self.get_logger().info(f'  Left stick X (axis {self.axis_0}) -> joint_0')
        self.get_logger().info(f'  Left stick Y (axis {self.axis_1}) -> joint_1')
    
    def joy_callback(self, msg: Joy):
        """
        Process joystick input and publish joint commands.
        PS3 Controller mapping:
        - Left stick X: controls joint_0
        - Left stick Y: controls joint_1
        """
        # Get joystick values
        if len(msg.axes) > max(self.axis_0, self.axis_1):
            input_0 = msg.axes[self.axis_0]
            input_1 = msg.axes[self.axis_1]

            target_pos_0 = input_0 * self.max_pos
            target_pos_1 = input_1 * self.max_pos
            
            # Update joint positions (integrate stick movement)
            self.joint_positions[0] = target_pos_0
            self.joint_positions[1] = target_pos_1

            self.joint_positions[3] = target_pos_0
            self.joint_positions[4] = target_pos_1

            self.joint_positions[6] = target_pos_0
            self.joint_positions[7] = target_pos_1

            self.joint_positions[9] = target_pos_0
            self.joint_positions[10] = target_pos_1

            # Publish commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = self.joint_positions
            self.pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()