---
sidebar_position: 3
description: Connecting AI agents to robot controllers using ROS 2
---

# Lesson 2: Bridging AI and Robot Controllers

## Learning Objectives

After completing this lesson, you will be able to:
- Understand how AI agents can interface with robotic systems through ROS 2
- Implement a bridge between AI decision-making and robot control
- Use ROS 2 topics and services to connect AI and robot systems
- Design appropriate message formats for AI-robot communication
- Apply design patterns for AI-robot integration

## Introduction

The integration of AI agents with robot controllers is a critical aspect of modern robotics. ROS 2 provides the communication infrastructure necessary to connect high-level AI decision-making systems with low-level robot control systems. This lesson covers various approaches to bridging AI and robot controllers using ROS 2's communication patterns.

## AI and Robotics Integration Patterns

### 1. Perception-Action Loop

AI agents typically follow a perception-action loop:
- Perceive the environment (sensor data)
- Process information and make decisions
- Act on the environment (send commands to robot)

In ROS 2, this looks like:

```
[Sensor Nodes] -> [Sensor Topics] -> [AI Node] -> [Command Topic] -> [Robot Controller]
                    ^                                    |
                    |------------------------------------+
```

### 2. Behavior Trees and Task Planning

AI systems often use behavior trees or task planners to organize complex robot behaviors. These systems can:

- Plan sequences of actions
- Handle unexpected situations
- Coordinate multiple robot capabilities

### 3. Reinforcement Learning Integration

Reinforcement learning agents can directly interact with the robot through ROS 2, receiving sensor observations and sending actions in a continuous learning loop.

## Practical Example: AI-Driven Navigation

Let's look at how an AI agent might control robot navigation:

### AI Node for Waypoint Planning

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class NavigationAI(Node):
    def __init__(self):
        super().__init__('navigation_ai')
        
        # Publisher for sending navigation goals
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        
        # Subscriber for robot status
        self.status_subscriber = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        
        # Plan navigation in a real scenario would use map data and current position
        self.waypoints = [
            {"x": 1.0, "y": 1.0, "theta": 0.0},
            {"x": 2.0, "y": 2.0, "theta": 1.57},
            {"x": 3.0, "y": 1.0, "theta": 3.14}
        ]
        
        self.current_waypoint = 0
        self.robot_status = "idle"
        
        # Plan navigation after a short delay
        self.timer = self.create_timer(2.0, self.plan_navigation)
    
    def status_callback(self, msg):
        self.robot_status = msg.data
        self.get_logger().info(f'Robot status: {self.robot_status}')
        
        # If robot reached waypoint, proceed to next one
        if self.robot_status == "reached_goal" and self.current_waypoint < len(self.waypoints) - 1:
            self.current_waypoint += 1
            self.send_next_waypoint()
    
    def plan_navigation(self):
        if self.waypoints:
            self.send_next_waypoint()
    
    def send_next_waypoint(self):
        if self.current_waypoint < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint]
            
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.position.x = waypoint['x']
            goal_msg.pose.position.y = waypoint['y']
            goal_msg.pose.position.z = 0.0
            
            # Simple orientation based on theta
            # In practice, you'd convert theta to quaternion
            goal_msg.pose.orientation.w = 1.0  # For simplicity
            
            self.goal_publisher.publish(goal_msg)
            self.get_logger().info(f'Sent goal to waypoint {self.current_waypoint}: ({waypoint["x"]}, {waypoint["y"]})')

def main(args=None):
    rclpy.init(args=args)
    ai_node = NavigationAI()
    
    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI-Controller Communication Patterns

### 1. High-Level Commands

AI systems often send high-level commands to the robot:

- "Go to location X"
- "Pick up object Y"
- "Perform task Z"

These commands are then processed by specialized controllers.

### 2. Direct Control

For more sophisticated control, AI systems might send direct control commands:

- Velocity commands
- Joint positions
- Torque values

### 3. Hybrid Approach

A hybrid approach combines high-level commands with feedback:

```
[AI Decision] -> [High-level Command] -> [Trajectory Planner] -> [Robot]
                    ^                                                   |
                    |------------------- [Feedback] ---------------------+
```

## Designing Appropriate Message Formats

When bridging AI and robots, consider:

1. **Semantic Meaning**: Messages should be meaningful to both AI and robot systems
2. **Abstraction Level**: Balance between high-level concepts and low-level control
3. **Error Handling**: Include information about success/failure of actions
4. **Timing**: Consider real-time requirements of the robot system

## Example: AI-Driven Manipulation

Here's an example of an AI agent controlling a robot manipulator:

### AI Node for Object Manipulation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point

class ManipulationAI(Node):
    def __init__(self):
        super().__init__('manipulation_ai')
        
        self.command_publisher = self.create_publisher(
            String, '/manipulation_command', 10)
        self.object_subscriber = self.create_subscription(
            Point, '/detected_object', self.object_callback, 10)
        
        self.state = "searching"  # searching, grasping, placing
    
    def object_callback(self, msg):
        if self.state == "searching":
            # AI decision: choose which object to pick up based on criteria
            self.get_logger().info(f'Detected object at ({msg.x}, {msg.y}, {msg.z})')
            
            # For this example, we'll just pick up the first object we see
            command = String()
            command.data = f"grasp {msg.x} {msg.y} {msg.z}"
            self.command_publisher.publish(command)
            self.state = "grasping"

def main(args=None):
    rclpy.init(args=args)
    ai_node = ManipulationAI()
    
    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Considerations for AI-Robot Integration

1. **Latency**: AI processing may introduce delays; plan accordingly
2. **Robustness**: AI systems can fail; have fallback behaviors
3. **Safety**: Implement safety checks between AI decisions and robot actions
4. **Learning**: Consider how to incorporate learning and adaptation
5. **Monitoring**: Implement monitoring to track AI-robot interaction performance

## Success Criteria

After completing this lesson, you should be able to:
- Design communication patterns between AI and robot controllers
- Implement nodes that bridge AI decision-making with robot control
- Select appropriate message formats for AI-robot communication
- Apply integration patterns to your own AI-robot systems

## Prerequisites

- Understanding of ROS 2 communication patterns (from ROS 2 Foundations module)
- Experience with Python ROS 2 nodes (from Lesson 1 of this module)
- Basic understanding of AI/ML concepts

## Citations

- [ROS 2 Navigation System](https://navigation.ros.org/)
- [Behavior Trees in Robotics and AI](https://arxiv.org/abs/1709.00084)
- Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/