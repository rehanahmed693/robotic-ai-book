---
sidebar_position: 3
description: Building a complete humanoid robot model using URDF
---

# Lesson 2: Building a Basic Humanoid Model

## Learning Objectives

After completing this lesson, you will be able to:
- Design a basic humanoid robot with appropriate link and joint structure
- Implement the kinematic chains for humanoid robot limbs
- Define joint limits appropriate for humanoid movement
- Validate and visualize a complete humanoid robot model
- Understand the challenges in humanoid robot modeling

## Introduction

Creating a humanoid robot model in URDF involves representing the human-like structure with appropriate links and joints. A basic humanoid typically includes:

- A torso/trunk
- A head
- Two arms with shoulders, elbows, and wrists
- Two legs with hips, knees, and ankles

When designing humanoid models, it's important to consider:
- The degrees of freedom required for desired motions
- Joint limits that reflect human-like capabilities
- Physical properties that make simulation realistic
- Simplifications that make the model computationally efficient

## Basic Humanoid Structure

A simple humanoid model might include these main components:

```
         head
           |
        neck
           |
         torso
        /     \
    shoulder  shoulder
      |         |
    elbow     elbow
      |         |
    wrist     wrist
      |         |
    (hands)   (hands)
       \         /
        hip    hip
          \    /
         (legs to feet)
```

For this lesson, we'll focus on the upper body (torso, head, and arms) to demonstrate the principles.

## Complete Humanoid URDF Example

Here's a complete example of a simple humanoid upper body:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.3 0.3 1.0 1.0"/>
  </material>
  <material name="red">
    <color rgba="1.0 0.3 0.3 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.3 1.0 0.3 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

  <!-- Torso (base link) -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.8"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.8"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <origin xyz="0 0 0.4" rpy="0 0 0"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.8" iyz="0" izz="0.6"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.85" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="20" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

  <!-- Right Shoulder -->
  <link name="right_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.2 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Right Shoulder Joint -->
  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="20" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

  <!-- Right Upper Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Elbow Joint -->
  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_upper_arm"/>
    <origin xyz="0 -0.1 0" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.5" effort="20" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

  <!-- Right Lower Arm -->
  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right Wrist Joint -->
  <joint name="right_wrist_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 -0.4 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Left Shoulder -->
  <link name="left_shoulder">
    <visual>
      <geometry>
        <box size="0.1 0.2 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Left Shoulder Joint -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.2 0 0.6" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="1.57" effort="20" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

  <!-- Left Upper Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Elbow Joint -->
  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_upper_arm"/>
    <origin xyz="0 -0.1 0" rpy="1.57 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.5" effort="20" velocity="1.0"/>
    <dynamics damping="1.0" friction="0.1"/>
  </joint>

  <!-- Left Lower Arm -->
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <origin xyz="0 -0.2 0" rpy="1.57 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Left Wrist Joint -->
  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 -0.4 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>
</robot>
```

## Key Design Considerations

### Joint Limits
When setting joint limits, consider:
- Human anatomical limits (e.g., elbow can't rotate 360°)
- Robot mechanical limits
- Simulation stability

### Coordinate Systems
Remember that in URDF:
- The origin of each link defines its local coordinate system
- Joint origins specify the position and orientation of the joint relative to the parent link
- The axis tag defines the axis of rotation/translation in the joint's coordinate system

### Inertial Properties
For simulation to behave realistically:
- Mass values should reflect the real-world robot
- Inertia tensors should be reasonably accurate
- Center of mass should be correctly positioned

## Validating Your Humanoid Model

You can validate your humanoid model using various ROS tools:

```bash
# Check if the URDF is well-formed
check_urdf humanoid.urdf

# Visualize the robot in RViz
ros2 run rviz2 rviz2

# Or use the simple visualization tool:
urdf_to_graphviz humanoid.urdf
```

## Success Criteria

After completing this lesson, you should be able to:
- Create a complete humanoid robot model in URDF
- Implement appropriate kinematic chains for humanoid limbs
- Apply realistic joint limits for humanoid movement
- Validate and visualize your humanoid model

## Prerequisites

- Understanding of URDF structure and joint definitions (from Lesson 1)
- Basic knowledge of 3D geometry and coordinate systems
- Understanding of ROS 2 concepts (from previous modules)

## Citations

- [ROS 2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Humanoid Robot Modeling Best Practices](https://ieeexplore.ieee.org/document/8460442)
- Open Robotics. (2022). ROS 2 Documentation. https://docs.ros.org/en/humble/