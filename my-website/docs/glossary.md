---
title: Glossary of Terms
sidebar_label: Glossary
description: Definitions of key terms used in digital twin robotics simulation
keywords: [glossary, terms, definitions, robotics, simulation, digital twin, gazebo, unity]
learning_objectives:
  - Understand key terminology in robotics simulation
  - Apply correct terminology when discussing digital twin concepts
  - Reference definitions for consistent communication
duration: 15
---

import LearningObjectives from '@site/src/components/LearningObjectives';
import DurationEstimator from '@site/src/components/DurationEstimator';

<LearningObjectives objectives={[
  'Understand key terminology in robotics simulation',
  'Apply correct terminology when discussing digital twin concepts',
  'Reference definitions for consistent communication'
]} />

<DurationEstimator minutes={15} activity="reading" />

# Glossary of Terms

This glossary provides definitions for key terms used throughout the digital twin robotics simulation educational content.

## A

**Accelerometer**: A sensor that measures proper acceleration (the acceleration felt by the sensor). In robotics, accelerometers are often used as part of an IMU to measure the robot's acceleration in 3D space.

**Allan Variance**: A statistical measure used to characterize the noise properties of precision instruments, particularly IMUs, across different time scales. It helps identify different noise sources like white noise, bias instability, and random walk.

## B

**Bias**: A systematic error in sensor measurements that remains consistent across measurements. In IMUs, bias refers to the offset from the true value that affects all measurements.

**Bounce (Restitution)**: A physics property that determines how "bouncy" an object is during collisions. Also known as the coefficient of restitution, it ranges from 0 (no bounce) to values greater than 1 (highly bouncy).

## C

**Collision Detection**: The computational problem of detecting when two or more objects come into contact with each other in a simulation environment.

**Continuous Collision Detection**: A method of detecting collisions between time steps, which prevents fast-moving objects from passing through each other (tunneling effect).

**Coordinate System**: A system to define positions and orientations in 3D space. Different simulation environments may use different conventions (e.g., right-handed vs. left-handed).

## D

**Digital Twin**: A virtual replica of a physical system that enables simulation, testing, and analysis of robot behaviors without real-world hardware.

**Discrete Collision Detection**: A method that checks for collisions only at specific time intervals, potentially missing collisions if objects move quickly.

**Damping**: The dissipation of energy that causes motion to decrease over time. In simulation, it's used to represent energy loss due to friction, air resistance, etc.

## G

**Gazebo**: A 3D simulation environment with physics engine and sensor capabilities for robotics development and testing.

**Gravity**: The force that attracts objects with mass toward each other. In simulation, it's typically set to 9.8 m/s² to match Earth's gravitational acceleration.

**Gyroscope**: A sensor that measures angular velocity around three axes. Part of the IMU, it measures how fast an object is rotating.

## I

**IMU (Inertial Measurement Unit)**: A device that measures and reports a body's specific force, angular rate, and sometimes the magnetic field surrounding the body, using a combination of accelerometers, gyroscopes, and magnetometers.

**Inertia**: The resistance of any physical object to any change in its velocity. In robotics simulation, it describes how mass is distributed in a 3D object.

## L

**LiDAR (Light Detection and Ranging)**: A remote sensing method that uses light in the form of a pulsed laser to measure distances to objects, creating precise, three-dimensional information about the shape of objects and the environment.

## M

**Magnetometer**: A sensor that measures magnetic fields. In an IMU, it provides orientation reference relative to Earth's magnetic field.

**Mass**: The amount of matter in an object, which affects its response to forces (F=ma).

**Middleware**: Software that provides common services and capabilities to applications outside of what's offered by the operating system, such as ROS for robotics.

## N

**Noise Density**: A specification for sensors (especially IMUs) that describes the noise level normalized to unit bandwidth, typically specified in the sensor datasheet.

**Normal Mapping**: A 3D graphics technique used to add detail to surfaces by perturbing the surface normals, creating the illusion of complex geometry without additional polygons.

## P

**PBR (Physically Based Rendering)**: A method of shading and rendering that simulates how light interacts with materials in the real world, creating more realistic visual results.

**Physics Engine**: Software that simulates physical interactions between objects, including gravity, collisions, and response forces.

**Point Cloud**: A set of data points in space, typically representing the external surface of an object. Point clouds are often created by 3D scanners or simulated sensors like LiDAR.

## R

**Ray Sensor**: A type of sensor simulation in Gazebo that works by casting rays and measuring distances to objects in the environment, commonly used to simulate LiDAR and laser range finders.

**Reflection**: The change in direction of a wavefront at an interface between two different media so that the wavefront returns into the medium from which it originated, important in realistic rendering.

**Restitution**: See "Bounce". A physics property that determines how "bouncy" an object is during collisions.

**RGB-D**: Refers to sensors that provide both color (RGB) and depth (D) information, like the Microsoft Kinect or Intel RealSense cameras.

**ROS (Robot Operating System)**: A flexible framework for writing robot software that provides services designed for a heterogeneous computer cluster.

## S

**SDF (Simulation Description Format)**: An XML-based format for describing objects and environments in simulation, used by Gazebo.

**Sensor Fusion**: The process of combining data from multiple sensors to produce more accurate and reliable information than could be obtained from any single sensor.

**Simplified Model**: A less complex representation of a more complex system, used for faster computation while maintaining essential characteristics.

**Simulation**: The imitation of the operation of a real-world process or system over time, particularly important in robotics for testing without physical hardware.

## U

**URDF (Unified Robot Description Format)**: An XML format for representing a robot model, including kinematic and dynamic properties, visual appearance, and collision properties.

**Unity**: A cross-platform game engine widely used for creating 3D simulations and visualizations in robotics and other fields.

## V

**Virtual Reality (VR)**: A simulated experience that can be similar to or completely different from the real world, often used for immersive robotics simulation and training.

**Visualization**: The representation of data or systems in graphical form, particularly important for understanding robotics simulation results.