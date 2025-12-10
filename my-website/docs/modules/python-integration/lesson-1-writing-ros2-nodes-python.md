---
sidebar_position: 2
description: Creating ROS 2 nodes using the rclpy Python client library
---

# Lesson 1: Writing ROS 2 Nodes in Python

## Learning Objectives

After completing this lesson, you will be able to:
- Set up the Python development environment for ROS 2
- Create ROS 2 nodes using the rclpy client library
- Implement publishers, subscribers, services, and clients in Python
- Use ROS 2 parameters and logging in Python nodes
- Structure Python code following ROS 2 best practices

## Introduction to rclpy

The `rclpy` package is the Python client library for ROS 2. It provides a Python API to interact with the ROS 2 middleware, allowing you to create nodes, publish and subscribe to topics, and provide and use services.

`rclpy` is built on top of `rcl` (ROS Client Library) and `rmw` (ROS Middleware), providing a Python-specific interface to the underlying ROS 2 infrastructure.

## Setting up the Environment

Before writing Python nodes for ROS 2, ensure you have:

1. ROS 2 Humble Hawksbill installed
2. Python 3.8 or higher
3. The ROS 2 environment sourced:

```bash
source /opt/ros/humble/setup.bash
```

## Creating Your First Node

Let's create a simple Python node:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Components Explained:

- `rclpy.init(args=args)` - Initializes the ROS 2 client library
- `MyNode()` - Creates an instance of your custom node class
- `rclpy.spin(node)` - Keeps the node running, processing callbacks
- `node.destroy_node()` - Properly cleans up node resources
- `rclpy.shutdown()` - Shuts down the ROS 2 client library

## Implementing Publishers in Python

Here's how to implement a publisher in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing Subscribers in Python

Here's how to implement a subscriber in Python:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Services in Python

Here's how to implement a service server and client in Python:

### Service Server
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    
    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        f'Result of add_two_ints: {1} + {2} = {response.sum}')
    
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Parameters

ROS 2 allows nodes to have parameters that can be configured at runtime:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('my_parameter', 'default_value')
        self.declare_parameter('count_threshold', 10)
        
        # Get parameter values
        self.param_value = self.get_parameter('my_parameter').value
        self.threshold = self.get_parameter('count_threshold').value
        
        self.get_logger().info(f'Parameter value: {self.param_value}')
        self.get_logger().info(f'Threshold: {self.threshold}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Python Nodes

1. Use proper exception handling with try/except blocks
2. Always clean up resources in a finally block
3. Follow Python naming conventions (PEP 8)
4. Use docstrings to document your code
5. Separate concerns into different classes/files when appropriate
6. Use appropriate Quality of Service (QoS) profiles for your application

## Success Criteria

After completing this lesson, you should be able to:
- Create a ROS 2 node using rclpy
- Implement publishers and subscribers in Python
- Create and use services in Python
- Work with parameters in Python nodes
- Follow best practices for Python ROS 2 development

## Prerequisites

- Basic Python programming knowledge
- Understanding of ROS 2 concepts (from ROS 2 Foundations module)
- ROS 2 Humble Hawksbill installed

## Citations

- [ROS 2 Documentation - Writing a Simple Python Publisher and Subscriber](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Documentation - Writing a Simple Python Service and Client](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/