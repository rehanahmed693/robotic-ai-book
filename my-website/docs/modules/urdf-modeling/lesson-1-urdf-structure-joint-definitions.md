---
sidebar_position: 2
description: Understanding URDF structure and joint definitions for humanoid robots
---

# Lesson 1: URDF Structure and Joint Definitions

## Learning Objectives

After completing this lesson, you will be able to:
- Understand the structure and components of URDF files
- Define links and joints for robot models
- Specify joint types, limits, and properties
- Create basic kinematic chains for robot models
- Validate URDF files using ROS tools

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and kinematic properties of a robot, including:

- Links: Rigid parts of the robot (e.g., chassis, links, grippers)
- Joints: Connections between links (e.g., rotational, prismatic)
- Visual and collision properties
- Inertial properties
- Transmissions and sensors

URDF is fundamental for robot simulation, visualization, and kinematic analysis.

## Basic URDF Structure

A basic URDF file consists of:
- XML declaration
- Robot tag with name
- Links (rigid parts)
- Joints (connections between links)

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  
  <!-- Joints -->
  <joint name="base_to_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
  </joint>
  
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>
</robot>
```

## Links in URDF

A link represents a rigid part of the robot. Each link can have:

- Visual properties: How it appears in visualization
- Collision properties: How it behaves in collision detection
- Inertial properties: Mass and moment of inertia for physics simulation

### Visual Properties
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
    <!-- or <cylinder radius="1" length="1"/> -->
    <!-- or <sphere radius="1"/> -->
    <!-- or <mesh filename="path_to_model.dae"/> -->
  </geometry>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
</visual>
```

### Collision Properties
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <box size="1 1 1"/>
  </geometry>
</collision>
```

### Inertial Properties
```xml
<inertial>
  <mass value="1.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
</inertial>
```

## Joints in URDF

Joints connect links and define how they move relative to each other. Common joint types include:

- `revolute`: Rotational joint with limited range
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement between links
- `floating`: 6 DOF (not commonly used)
- `planar`: Movement on a plane (not commonly used)

### Joint Definition
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="1.0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

### Joint Types Explained:

1. **Revolute Joint**: A rotational joint with limited range of motion
   - Use for elbow, knee, or neck joints on humanoid robots
   - Requires `limit` tag with lower and upper bounds

2. **Continuous Joint**: A rotational joint without range limits
   - Use for wheels or rotating sensors
   - No `limit` tag needed

3. **Prismatic Joint**: A sliding joint with limited range
   - Use for telescoping parts or drawer mechanisms
   - Requires `limit` tag with lower and upper bounds

4. **Fixed Joint**: A joint that doesn't move
   - Use to connect parts that should always move together
   - No `limit` tag needed

## Creating a Simple Robot Model

Let's create a simple robot with a base and one moving joint:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Arm Link -->
  <link name="arm_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base and arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>  <!-- Rotate around Y axis -->
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Kinematic Chains

A kinematic chain is a series of links connected by joints. In humanoid robots, you'll often create chains for:

- Arm: shoulder → elbow → wrist → hand
- Leg: hip → knee → ankle → foot
- Spine: multiple vertebrae

When creating kinematic chains, remember:
- Each joint connects exactly two links
- The chain typically starts with a fixed "base" link
- Joint origins define the position and orientation of the joint relative to the parent link

## Validating URDF Files

You can validate your URDF files using ROS tools:

```bash
# Check URDF syntax and structure
check_urdf my_robot.urdf

# Parse URDF and show joint information
urdf_to_graphviz my_robot.urdf
```

## Success Criteria

After completing this lesson, you should be able to:
- Create a valid URDF file with links and joints
- Define different joint types with appropriate parameters
- Specify visual, collision, and inertial properties
- Validate URDF files using ROS tools

## Prerequisites

- Basic understanding of XML syntax
- Basics of 3D geometry and coordinate systems
- Understanding of ROS 2 concepts (from previous modules)

## Citations

- [URDF/XML Reference](http://wiki.ros.org/urdf/XML)
- [ROS 2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/