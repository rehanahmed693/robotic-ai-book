# Basic Scene Creation Tutorial

## Overview
In this lesson, you will learn how to create a basic simulation scene in Isaac Sim. This is the first step to building any robotics simulation, where you'll define the environment in which your robot will operate.

## Learning Objectives
After completing this lesson, you will be able to:
- Launch Isaac Sim and navigate the interface
- Create a new simulation scene
- Add basic objects to your environment
- Position objects in the 3D space
- Save and manage your scene

## Prerequisites
- Isaac Sim installed and properly configured
- Basic understanding of 3D environments (helpful but not required)

## Step 1: Launch Isaac Sim
1. Open Isaac Sim from your desktop or command line
2. The application will start with the default scene
3. You'll see the main interface with the viewport, stage panel, and property panel

## Step 2: Create a New Scene
1. Go to `File` → `New` in the top menu bar
2. This clears the current scene and creates an empty workspace
3. You'll see a grid representing the ground plane in the viewport

## Step 3: Setting up the Environment
### Adding a Ground Plane
1. In the Stage panel (left side), right-click in an empty area
2. Select `Create` → `Ground Plane`
3. A flat surface will appear in your scene

### Adding a Basic Room
1. Right-click in the Stage panel again
2. Select `Create` → `Physics` → `Fixed Joint` → `Rigid Body` → `Cuboid`
3. This creates a basic cube that can serve as a wall
4. With the cube selected, go to the Property panel (on the right)
5. Under `Xform`, change the `Scale` values to create a wall (e.g., (5, 0.1, 3) for a floor or (0.1, 3, 5) for a wall)
6. Change the `Position` to place the wall appropriately (e.g., (2.5, 1.5, 0) for a wall along the x-axis)

### Creating Multiple Walls
1. Repeat the process to create 4 walls to form a room
2. Adjust positions and scales to form an enclosed space:
   - Wall 1: Position (2.5, 1.5, 0), Scale (5, 3, 0.1)
   - Wall 2: Position (-2.5, 1.5, 0), Scale (5, 3, 0.1)
   - Wall 3: Position (0, 1.5, 2.5), Scale (0.1, 3, 5)
   - Wall 4: Position (0, 1.5, -2.5), Scale (0.1, 3, 5)

## Step 4: Adding Objects to Your Scene
### Adding a Simple Object
1. Right-click in the Stage panel
2. Select `Create` → `Mesh Primitives` → `Cylinder`
3. Position the cylinder in your scene:
   - In the Property panel, under `Xform`, change the `Position` to something like (0, 0.5, 0)
   - Adjust the `Scale` if needed (e.g., (0.5, 0.5, 0.5))

### Adding Visual Materials
1. In the Content tab (usually on the left), navigate to `Isaac/Assets/Materials`
2. Drag a material (like a colored material) directly onto your object in the viewport
3. The object will now have the selected material appearance

## Step 5: Physics Setup
1. Select your object in the Scene
2. In the Property panel, ensure `physics:rigid_body` is enabled
3. Set the `mass` property if you want the object to interact physically
4. Adjust friction and restitution for realistic interactions:
   - `physics:friction` (typically 0.1-1.0)
   - `physics:restitution` (bounciness, typically 0.0-0.5)

## Step 6: Viewing and Navigating Your Scene
### Navigation Controls
- **Orbit**: Hold `Alt` + left mouse button, then drag
- **Pan**: Hold `Alt` + right mouse button, then drag
- **Zoom**: Scroll the mouse wheel or hold `Alt` + middle mouse button and drag

### Changing Camera Views
- Use the `F` key to focus on selected objects
- Use the view cube in the top right corner to look at the scene from different angles

## Step 7: Saving Your Scene
1. Go to `File` → `Save As`
2. Choose a location to save your scene file (typically with `.usd` or `.usda` extension)
3. Give it a meaningful name like `basic_room_scene.usd`

## Troubleshooting Tips
- **Objects falling through the ground**: Check that both the ground and your object have physics properties enabled
- **Objects not appearing**: Verify that the visibility property is set to true
- **Performance issues**: Reduce the complexity of objects or the number of them in the scene

## Summary
You've now created a basic scene with walls and objects in Isaac Sim. This foundation allows you to build more complex environments for your robotics simulations. In the next lesson, we'll explore configuring physics properties in more detail.

## Next Steps
Continue to the next lesson to learn about physics configuration and how to make your simulation behave realistically.