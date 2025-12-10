---
sidebar_position: 2
description: Understanding the ROS 2 middleware architecture
---

# Lesson 1: ROS 2 Architecture

## Learning Objectives

After completing this lesson, you will be able to:
- Explain the ROS 2 middleware architecture and its role in robotics
- Identify the key components of the ROS 2 architecture (DDS, nodes, topics, etc.)
- Understand the difference between ROS 1 and ROS 2 architecture
- Describe the concept of ROS 2 as a "nervous system" for robots

## Overview

ROS 2 (Robot Operating System 2) is a middleware framework designed to facilitate communication between different software components of a robot system. Unlike ROS 1, which used a centralized master architecture, ROS 2 uses Data Distribution Service (DDS) to provide a distributed communication infrastructure.

The ROS 2 middleware architecture enables:

- Scalable, distributed systems
- Fault tolerance
- Support for multi-robot systems
- Integration with embedded systems
- Real-time capabilities

## Key Components of ROS 2 Architecture

### Data Distribution Service (DDS)

DDS (Data Distribution Service) is the underlying communication middleware that ROS 2 uses. It provides:

- Publisher/subscriber communication pattern
- Service/client communication pattern
- Request/reply communication pattern
- Discovery mechanisms
- Quality of Service (QoS) policies

DDS implementations include:
- Fast DDS (formerly Fast RTPS)
- Cyclone DDS
- RTI Connext DDS
- Eclipse Zenoh

### Nodes

A node is a process that performs computation. In ROS 2:
- Nodes are the basic computational elements of a ROS program
- Each node communicates with other nodes using the ROS 2 communication layer
- Nodes can publish or subscribe to topics, provide services, or create clients

### Topics and Messages

- Topics are named buses over which nodes exchange messages
- Messages are the data packets sent on topics
- Messages are defined in `.msg` files and contain a list of fields with specific data types

### Services

- Services provide a request/reply communication pattern
- A service has a request message and a response message
- Services are defined in `.srv` files

### Actions

- Actions are similar to services but designed for long-running operations
- They support goals, feedback, and results
- Actions are defined in `.action` files

## ROS 2 as a "Nervous System"

The concept of ROS 2 as a "nervous system" for robots refers to how it enables various components of a robot to communicate effectively:

- **Sensory Input**: Sensors publish data to topics
- **Processing Centers**: Processing nodes subscribe and react to sensor data
- **Motor Output**: Actuator nodes receive commands and execute robot behavior
- **Feedback Loops**: Multiple nodes interact to create complex behavior

This architecture allows for modular development where each node can be developed and tested independently.

## Practical Example: Simple Node Architecture

Let's consider a simple robot navigation system:

```
[Sensor Node] -> [Sensor Data Topic] -> [Navigation Node] -> [Command Topic] -> [Actuator Node]
```

In this system:
- The Sensor Node publishes sensor data (e.g., LIDAR scans)
- The Navigation Node subscribes to sensor data and publishes navigation commands
- The Actuator Node receives navigation commands and controls the robot's motors

## Success Criteria

After completing this lesson, you should be able to:
- Explain the difference between ROS 1 and ROS 2 architecture
- Identify the main components of ROS 2 and their functions
- Describe how ROS 2 enables modular robot development
- Apply the "nervous system" concept to explain ROS 2's role in robotics

## Prerequisites

- Basic understanding of robotics concepts
- Familiarity with Linux command line
- Basic knowledge of distributed systems

## Citations

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- Quigley, M., Gerkey, B., & Smart, W. (2009). The ROS: a middleware for robot software development.
- Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/