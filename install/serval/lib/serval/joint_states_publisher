#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64MultiArray

class JointStatesPublisher(Node):
    def __init__(self):
        super().__init__('joint_states_publisher')
        
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Subscribe to joint commands from teleop
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/joint_commands',
            self.joint_commands_callback,
            10
        )
        
        # Store current joint positions
        self.joint_positions = [0.0, 0.0, 0.0]
        
        self.timer = self.create_timer(0.05, self.publish_joint_states)  # 20 Hz
        
        self.get_logger().info('Joint states publisher started')
    
    def joint_commands_callback(self, msg: Float64MultiArray):
        """Update joint positions from teleop commands"""
        self.joint_positions = list(msg.data)
    
    def publish_joint_states(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Joint names from URDF (4 legs × 3 joints per leg)
        msg.name = [
            'hip_fr', 'upper_leg_fr', 'lower_leg_fr',
            'hip_fl', 'upper_leg_fl', 'lower_leg_fl',
            'hip_br', 'upper_leg_br', 'lower_leg_br',
            'hip_bl', 'upper_leg_bl', 'lower_leg_bl',
        ]
        
        # Use commanded positions (pad with zeros if fewer commands received)
        positions = list(self.joint_positions) + [0.0] * (12 - len(self.joint_positions))
        msg.position = positions[:12]
        msg.velocity = [0.0] * 12
        msg.effort = [0.0] * 12
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
