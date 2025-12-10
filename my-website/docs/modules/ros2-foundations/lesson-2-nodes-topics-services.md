---
sidebar_position: 3
description: Core communication patterns in ROS 2 - nodes, topics, and services
---

# Lesson 2: Nodes, Topics, and Services

## Learning Objectives

After completing this lesson, you will be able to:
- Create and run ROS 2 nodes in different programming languages
- Implement publisher and subscriber communication patterns
- Create and use services for request/response interactions
- Understand Quality of Service (QoS) settings and their impact
- Build a simple communication system with nodes, topics, and services

## Introduction

ROS 2 provides several communication patterns to enable different types of interactions between nodes. Understanding these communication patterns is essential for designing effective robot software architectures. This lesson covers the three primary communication patterns: topics (publish/subscribe), services (request/reply), and briefly introduces actions.

## Nodes

A node is the fundamental building block of a ROS 2 system. Each node performs specific computation and communicates with other nodes through the ROS 2 communication layer.

### Creating a Node

In Python, a node is created by subclassing the `rclpy.Node` class:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    rclpy.spin(minimal_node)

    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- `rclpy.init(args=args)` initializes the ROS 2 client library
- `MinimalNode()` creates an instance of your custom node class
- `rclpy.spin(minimal_node)` keeps the node running and processing callbacks
- `minimal_node.destroy_node()` properly cleans up the node resources
- `rclpy.shutdown()` shuts down the ROS 2 client library

## Topics and Publishers/Subscribers

Topics enable asynchronous, many-to-many communication between nodes. Publishers send messages to a topic, and subscribers receive messages from that topic.

### Publisher Example

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
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- `self.create_publisher(String, 'topic', 10)` creates a publisher that publishes `String` messages to the topic named 'topic' with a queue size of 10
- `self.create_timer(timer_period, self.timer_callback)` creates a timer that calls `timer_callback` every 0.5 seconds
- `self.publisher_.publish(msg)` publishes the message to the topic
- Quality of Service (QoS) is specified with the queue size (10 messages)

### Subscriber Example

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
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- `self.create_subscription(String, 'topic', self.listener_callback, 10)` creates a subscription to the 'topic' that expects `String` messages and uses a queue size of 10
- `self.listener_callback` is the callback function that executes when a message is received
- The callback receives the message as a parameter (`msg`)
- Quality of Service (QoS) is again specified with the queue size

## Services

Services provide synchronous, request/reply communication between nodes. A service has a server node that provides the service and client nodes that make requests to the service.

### Service Server Example

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
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- `self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)` creates a service with the name 'add_two_ints' using the AddTwoInts service definition
- The service definition (`.srv` file) contains the request and response structures
- The callback receives both the `request` and `response` objects
- The callback must return a response object

### Service Client Example

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
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
        'Result of add_two_ints: for %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation:**
- `self.create_client(AddTwoInts, 'add_two_ints')` creates a client for the 'add_two_ints' service
- `self.cli.wait_for_service(timeout_sec=1.0)` waits for the service to become available
- `self.cli.call_async(self.req)` makes an asynchronous service call
- `rclpy.spin_until_future_complete(self, self.future)` waits for the response
- The result is obtained using `self.future.result()`

## Quality of Service (QoS)

Quality of Service profiles allow you to specify policies for reliability, durability, liveliness, and history of messages. These policies help ensure that messages are delivered according to your application's requirements.

Common QoS profiles:
- **Reliability**: Best effort vs reliable
- **Durability**: Volatile vs transient local
- **History**: Keep last vs keep all

## Practical Exercise: Publisher-Subscriber Pair

Let's implement a simple publisher-subscriber pair that demonstrates basic ROS 2 communication:

1. Create a publisher node that publishes messages with a counter
2. Create a subscriber node that receives and logs the messages
3. Run both nodes and observe the communication

## Success Criteria

After completing this lesson, you should be able to:
- Create publisher and subscriber nodes in ROS 2
- Implement service servers and clients
- Apply appropriate QoS policies for your use case
- Demonstrate successful communication between nodes using topics and services

## Prerequisites

- Understanding of ROS 2 architecture (from Lesson 1)
- Basic Python programming knowledge
- ROS 2 Humble Hawksbill installed

## Citations

- [ROS 2 Documentation - Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Documentation - Services](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/