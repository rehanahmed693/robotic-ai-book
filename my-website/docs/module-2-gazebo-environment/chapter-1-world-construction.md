---
title: Chapter 1 - World Construction
sidebar_label: World Construction
description: Building custom simulation environments in Gazebo for robotic applications
keywords: [gazebo, world building, simulation, robotics, environment]
learning_objectives:
  - Create custom simulation worlds using Gazebo tools
  - Configure environment properties and lighting
  - Add and position objects in the simulation world
duration: 30
---

import LearningObjectives from '@site/src/components/LearningObjectives';
import DurationEstimator from '@site/src/components/DurationEstimator';
import Assessment from '@site/src/components/Assessment';

<LearningObjectives objectives={[
  'Create custom simulation worlds using Gazebo tools',
  'Configure environment properties and lighting',
  'Add and position objects in the simulation world'
]} />

<DurationEstimator minutes={30} activity="reading" />

# Chapter 1: World Construction

Creating effective simulation environments is a critical skill for robotics development. This chapter will guide you through building custom worlds in Gazebo, from basic terrain to complex scenarios with multiple objects and properties.

## 1. Gazebo World Structure

A Gazebo world is defined using SDF (Simulation Description Format), which is an XML-based format. Key elements of a world include:

- **World element**: The root element containing all world properties
- **Physics engine**: Defines the physics engine used for simulation (ODE, Bullet, SimBody)
- **Scene properties**: Lighting, shadows, and visual appearance
- **Models**: Objects and robots within the world
- **Lights**: Light sources in the environment
- **Ground plane**: The default ground surface

## 2. Creating a Basic World

To create a basic world, start with the following SDF structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="my_world">
    <!-- Physics engine configuration -->
    <physics name="default" type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Scene properties -->
    <scene>
      <ambient>0.4 0.4 0.4</ambient>
      <background>0.7 0.7 0.7</background>
    </scene>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun light -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Additional models will go here -->
  </world>
</sdf>
```

## 3. Adding Models to Your World

Models can be added in several ways:
- Using built-in models from the Gazebo Model Database
- Creating custom models in URDF/SDF format
- Placing models using the Gazebo GUI

When adding models, consider:
- Position and orientation in the world
- Physical properties (mass, friction, etc.)
- Visual properties (color, texture, etc.)

## 4. Environment Customization

Customize your environment by adjusting:
- **Lighting**: Direction, intensity, and color of light sources
- **Materials**: Surface properties and textures
- **Terrain**: Height maps for creating realistic landscapes
- **Weather**: Simulation of atmospheric conditions (in newer versions)

## 5. Performance Considerations

When building complex worlds, keep performance in mind:
- Simplify collision meshes when possible
- Use fewer light sources
- Limit the number of complex models in a single view
- Optimize textures and materials

<DurationEstimator minutes={20} activity="exercise" />

## Hands-on Exercise: Environment Construction (Task T026)

Now let's create your first custom Gazebo world:

1. Open Gazebo and select "Insert" from the menu
2. Add a ground plane to your world
3. Insert a simple model (e.g., a box, sphere, or cylinder)
4. Position the object in your world using the move tool
5. Adjust the object's properties (size, color)
6. Add a second object and set up a simple scene
7. Test the physics by adding a simple robot or using the physics properties

Save your world file as an SDF file for future use.

<Assessment 
  type="multiple-choice" 
  question="What is the primary purpose of the <physics> element in a Gazebo world file?" 
  options={[
    "To define the visual appearance of the world",
    "To configure the physics engine and properties like gravity",
    "To add lighting to the simulation",
    "To specify the camera view"
  ]} 
  correctAnswer="To configure the physics engine and properties like gravity" 
  explanation="The <physics> element configures the physics engine used by Gazebo, including properties like gravity, step size, and engine-specific parameters." 
/>

<Assessment 
  type="multiple-choice" 
  question="Which of the following is NOT a valid approach to add models to a Gazebo world?" 
  options={[
    "Using models from the Gazebo Model Database",
    "Creating custom models in URDF/SDF format",
    "Using the Gazebo GUI to place models",
    "Adding models directly through JavaScript"
  ]} 
  correctAnswer="Adding models directly through JavaScript" 
  explanation="While Gazebo can be controlled via JavaScript through GzWeb, the standard approaches are using the model database, custom model files, or the GUI. Direct JavaScript model addition isn't a standard approach." 
/>

<Assessment 
  type="short-answer" 
  question="Explain two important performance considerations when building complex Gazebo worlds and why they matter for robotics simulation." 
  correctAnswer="Two important performance considerations are: 1) Simplifying collision meshes - complex collision meshes require more computation and can slow down physics simulation, which is critical for real-time robotics applications. 2) Limiting the number of complex models in a single view - too many complex models can overwhelm the graphics and physics engines, causing lag that would affect the realism of robot control and sensor processing." 
  explanation="Two important performance considerations are: 1) Simplifying collision meshes - complex collision meshes require more computation and can slow down physics simulation, which is critical for real-time robotics applications. 2) Limiting the number of complex models in a single view - too many complex models can overwhelm the graphics and physics engines, causing lag that would affect the realism of robot control and sensor processing." 
/>