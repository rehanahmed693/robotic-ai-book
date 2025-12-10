# Physics Configuration Tutorial

## Overview
In this lesson, you will learn how to configure physics properties for objects in Isaac Sim. Proper physics configuration is essential for creating realistic simulations that accurately reflect real-world behavior.

## Learning Objectives
After completing this lesson, you will be able to:
- Configure gravity and other physics properties for an environment
- Set material properties for objects (friction, restitution)
- Understand the PhysX engine parameters and their effects
- Create realistic physical interactions between objects

## Prerequisites
- Basic scene creation skills (completed the previous lesson)
- Understanding of fundamental physics concepts (mass, friction, etc.)

## Step 1: Understanding Isaac Sim Physics
Isaac Sim uses NVIDIA's PhysX engine for physics simulation. Key parameters include:
- **Gravity**: The constant acceleration applied to all objects
- **Friction**: Resistance to motion when objects slide against each other
- **Restitution**: Bounciness or elasticity of objects
- **Density/Mass**: How heavy objects are

## Step 2: Configuring Global Physics Properties
### Setting Gravity
1. In the Stage panel, find the root node (`World` or similar)
2. In the Property panel, look for the `Physics` section
3. Find `defaultPhysicsScene` and expand it
4. Adjust the `gravity` vector:
   - Default is usually (0, 0, -9.81) for Earth gravity
   - The z-component is typically negative (pointing down)
   - Change values to simulate different gravitational environments (e.g., moon: -1.62)

### Configuring Physics Scene Properties
1. In the `defaultPhysicsScene`, you can also set:
   - `timeStep`: The simulation time step (smaller = more accurate but slower)
   - `solverType`: How the physics solver handles collisions
   - `maxSubSteps`: Number of sub-steps for collision resolution

## Step 3: Setting Object Physics Properties
### Adding Physics to Objects
1. Select an object in your scene
2. In the Property panel, right-click in an empty area under the property list
3. Select `Add Property` → `Physics` → `rigid_body`
4. This makes the object physically interactive

### Configuring Mass and Density
1. With a rigid body selected, look for `physics:mass` in the Property panel
2. Set the mass directly, or use `physics:density` to calculate mass based on volume
3. For complex objects, consider using `physics:massAPI` and `physics:bodyAPI` for fine-grained control

### Setting Friction Properties
1. Under physics properties, find `physics:friction` section
2. Set `physics:staticFriction`: Friction when the object is at rest (typically 0.0-2.0)
3. Set `physics:dynamicFriction`: Friction when the object is moving (typically 0.0-2.0, lower than static)
4. Example values:
   - Ice: 0.03-0.1 
   - Wood on wood: 0.2-0.5
   - Rubber on concrete: 0.6-0.85

### Setting Restitution (Bounciness)
1. Find `physics:restitution` in the Property panel
2. Value ranges from 0.0 (no bounce) to 1.0 (perfectly elastic)
3. Example values:
   - Clay/clay: ~0.0
   - Wood/wood: ~0.3
   - Tennis ball/concrete: ~0.8
   - Superball/concrete: ~0.9

## Step 4: Creating Physical Materials
### Using Material Assets
1. In the Content tab, navigate to `Isaac/Assets/Materials/PhysicsMaterials`
2. These materials combine visual appearance with physical properties
3. Drag and drop the material onto your object in the viewport or stage
4. Adjust properties as needed using the Property panel

### Creating Custom Physics Materials
1. Right-click in the Stage panel
2. Select `Create` → `Physics` → `Material`
3. In the Property panel, set:
   - `physics:staticFriction`
   - `physics:dynamicFriction`
   - `physics:restitution`

## Step 5: Advanced Physics Concepts
### Collision Filtering
1. Use collision groups and filter to control which objects interact
2. Set `physics:collisionGroups` and `physics:collisionFilter` properties

### Joints and Constraints
1. Select two objects that should be connected
2. Right-click → `Create` → `Physics` → `Joints`
3. Choose the appropriate joint type:
   - Fixed: Objects remain rigidly connected
   - Revolute: One rotational degree of freedom
   - Prismatic: One translational degree of freedom
   - Spherical: Ball-and-socket joint

### Kinematic Objects
1. Set an object as kinematic if it should move but not respond to physics forces
2. Find `physics:kinematic` in the Property panel and enable it
3. Use for objects like moving platforms or controllable robots

## Step 6: Testing Your Physics Configuration
### Running the Simulation
1. Press the play button in the top toolbar
2. Watch how objects behave with the physics properties you've set
3. Pause the simulation with the stop button
4. Adjust properties and run again to fine-tune

### Debugging Physics
1. Enable physics visualization:
   - Go to `Window` → `Physics` → `Physics Debug`
   - This shows collision shapes and forces
2. Check that objects don't fall through each other
3. Verify that objects behave realistically when interacting

## Step 7: Physics Optimization Tips
### Performance Considerations
- Use compound colliders for complex shapes rather than high-poly meshes
- Simplify collision geometry where high precision isn't needed
- Adjust `timeStep` and `maxSubSteps` for balance between accuracy and performance
- Limit the number of active physics objects if experiencing performance issues

### Accuracy Considerations
- Use smaller timeSteps for more accurate physics (but slower simulation)
- Increase solver iterations for more stable joint constraints
- Consider using continuous collision detection for fast-moving objects

## Troubleshooting Common Issues
- **Objects sinking into surfaces**: Check that both objects have physics properties and mass
- **Unrealistic bouncing**: Reduce restitution values
- **Objects sliding too much**: Increase friction values
- **Simulation instability**: Decrease timeStep or adjust solver parameters

## Summary
You've now configured physics properties for your Isaac Sim environment, allowing objects to interact realistically. These skills are crucial for creating accurate simulations for robotics development and testing. In the next lesson, we'll explore sensor simulation.

## Next Steps
Continue to the next lesson to learn about sensor simulation in Isaac Sim and how to configure various sensor types that are essential for robot perception.