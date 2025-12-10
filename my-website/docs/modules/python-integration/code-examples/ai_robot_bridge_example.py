# AI-to-Robot Control Bridge Example
# This demonstrates how an AI system can interface with robot controllers

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import random
import math

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge')
        
        # Robot interface publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Sensor data subscribers
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.robot_status_subscriber = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        # AI decision timer
        self.ai_timer = self.create_timer(1.0, self.ai_decision_cycle)
        
        # Robot state
        self.laser_data = None
        self.robot_status = "idle"
        self.current_task = "explore"
        
        self.get_logger().info('AI Bridge Node initialized')

    def scan_callback(self, msg):
        self.laser_data = msg
        # Process sensor data for AI decision making
        self.get_logger().info(f'Received laser scan with {len(msg.ranges)} range measurements')

    def status_callback(self, msg):
        self.robot_status = msg.data
        self.get_logger().info(f'Robot status updated: {self.robot_status}')

    def ai_decision_cycle(self):
        # Simple AI logic to decide next action
        if self.current_task == "explore":
            self.explore_behavior()
        elif self.current_task == "navigate":
            self.navigation_behavior()
        elif self.current_task == "avoid":
            self.avoidance_behavior()

    def explore_behavior(self):
        """Simple exploration behavior"""
        if self.laser_data and len(self.laser_data.ranges) > 0:
            # Look for the clearest direction
            mid_idx = len(self.laser_data.ranges) // 2
            front_clear = self.laser_data.ranges[mid_idx] > 1.0
            
            if front_clear:
                # Move forward
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.3
                cmd_msg.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd_msg)
                self.get_logger().info('Exploring: Moving forward')
            else:
                # Turn in a random direction to find clear path
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = random.choice([-0.5, 0.5])  # Turn left or right
                self.cmd_vel_publisher.publish(cmd_msg)
                self.get_logger().info('Exploring: Turning to find clear path')

    def navigation_behavior(self):
        """Navigate to a specific goal"""
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 5.0
        goal_msg.pose.position.y = 5.0
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        
        self.goal_publisher.publish(goal_msg)
        self.get_logger().info('Navigation: Sent goal to (5,5)')

    def avoidance_behavior(self):
        """Avoid obstacles using sensor data"""
        if self.laser_data and len(self.laser_data.ranges) > 0:
            # Get readings from different sectors
            left_sector = self.laser_data.ranges[0:len(self.laser_data.ranges)//3]
            front_sector = self.laser_data.ranges[len(self.laser_data.ranges)//3:2*len(self.laser_data.ranges)//3]
            right_sector = self.laser_data.ranges[2*len(self.laser_data.ranges)//3:]
            
            # Find minimum distances in each sector
            left_min = min([r for r in left_sector if not math.isinf(r)], default=float('inf'))
            front_min = min([r for r in front_sector if not math.isinf(r)], default=float('inf'))
            right_min = min([r for r in right_sector if not math.isinf(r)], default=float('inf'))
            
            cmd_msg = Twist()
            if front_min < 0.8:  # Obstacle too close in front
                # Priority: turn toward the clearer direction
                if right_min > left_min:
                    cmd_msg.angular.z = -0.5  # Turn right
                else:
                    cmd_msg.angular.z = 0.5   # Turn left
            else:
                cmd_msg.linear.x = 0.3  # Move forward if path is clear
            
            self.cmd_vel_publisher.publish(cmd_msg)
            self.get_logger().info(f'Obstacle avoidance: L:{left_min:.2f}, F:{front_min:.2f}, R:{right_min:.2f}')

def main(args=None):
    rclpy.init(args=args)
    ai_node = AIBridgeNode()
    
    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()