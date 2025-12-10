# Python Node Implementation Example
# This demonstrates how to create a ROS 2 node using rclpy

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create a publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a subscriber for sensor data
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Timer for robot behavior
        self.timer = self.create_timer(0.1, self.robot_behavior)
        
        self.obstacle_detected = False
        self.get_logger().info('Robot Controller Node has started')

    def scan_callback(self, msg):
        # Check if there's an obstacle directly in front
        if len(msg.ranges) > 0:
            front_distance = msg.ranges[len(msg.ranges) // 2]  # Middle value
            self.obstacle_detected = front_distance < 1.0  # Obstacle within 1 meter
            self.get_logger().info(f'Front distance: {front_distance:.2f}m, Obstacle: {self.obstacle_detected}')

    def robot_behavior(self):
        cmd_msg = Twist()
        
        if self.obstacle_detected:
            # Turn right if obstacle detected
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = -0.5
            self.get_logger().info('Turning right to avoid obstacle')
        else:
            # Move forward if no obstacle
            cmd_msg.linear.x = 0.5
            cmd_msg.angular.z = 0.0
            self.get_logger().info('Moving forward')
        
        # Publish the command
        self.cmd_vel_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()