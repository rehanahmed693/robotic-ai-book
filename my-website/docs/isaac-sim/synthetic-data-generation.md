# Synthetic Data Generation Tutorial

## Overview
In this lesson, you will learn how to generate synthetic data in Isaac Sim. Synthetic data is crucial for training AI models when real-world data is scarce, expensive to collect, or dangerous to obtain. Isaac Sim provides powerful tools for generating diverse, labeled datasets that can be used for computer vision, perception, and other AI applications.

## Learning Objectives
After completing this lesson, you will be able to:
- Configure synthetic data generation pipelines in Isaac Sim
- Generate labeled datasets with various annotations
- Use domain randomization to increase dataset diversity
- Export datasets in standard formats for AI training

## Prerequisites
- Understanding of Isaac Sim sensors and environment creation
- Basic knowledge of machine learning and computer vision concepts

## Step 1: Understanding Synthetic Data Generation
Synthetic data generation in Isaac Sim involves:
- Creating diverse simulation scenarios
- Capturing sensor data (images, point clouds, etc.)
- Generating automatic annotations (object detection boxes, segmentation masks, etc.)
- Varying environmental conditions to improve model robustness

## Step 2: Setting Up a Data Generation Scene
### Creating a Diverse Environment
1. Create a base scene with your robot and sensors
2. Add various objects that represent what you want to detect/train on
3. Consider multiple lighting conditions and times of day
4. Include different textures, colors, and materials

### Adding Annotatable Objects
1. Ensure all objects you want to detect are properly labeled
2. Use the `omni.kit.widget.layers` extension to create object prim hierarchy
3. Add semantic labels to objects using the Semantic Schema extension:
   - Select an object
   - Right-click → `Add` → `Semantic Schema`
   - Set the `semanticLabel` to a meaningful name (e.g., "chair", "table", "robot")

## Step 3: Configuring Data Acquisition Tools
### Using Isaac Sim Acquisition Tools
1. Enable the Isaac Acquisition extension: `Window` → `Isaac Lab` → `Acquisition`
2. This provides tools for capturing and organizing datasets

### Setting Up Data Writers
For RGB and depth data:
1. Go to `Window` → `Isaac Lab` → `Task Manager`
2. Create a new task for data collection
3. Configure the data writers for your desired output format

## Step 4: Creating Synthetic Annotations
### Semantic Segmentation
1. Isaac Sim can automatically generate semantic segmentation masks
2. Enable the segmentation view in the viewport:
   - Go to `View` → `Render Settings` → `Semantic Segmentation`
3. The semantic labels you added to objects will be used for segmentation masks

### 2D Bounding Boxes
1. For object detection tasks, Isaac Sim can generate 2D bounding boxes
2. Use the `Bbox2DAnnotator` extension which generates labeled rectangular boxes around objects

### 3D Oriented Bounding Boxes
1. For 3D detection tasks, use the `OrientedBboxAnnotator`
2. This creates 3D bounding boxes with position and orientation in 3D space

### Point Cloud Data
1. Use LiDAR sensors to generate point cloud data
2. The generated point clouds can be annotated with semantic information

## Step 5: Implementing Domain Randomization
Domain randomization helps make models trained on synthetic data more transferable to the real world:

### Texture Randomization
1. Create a material library with various textures
2. Use USD Variants to switch between different material appearances
3. Randomize textures during data collection

### Lighting Randomization
1. Add multiple light sources to your scene
2. Randomize:
   - Position and direction of lights
   - Color temperature
   - Intensity
   - Shadow properties

### Object Placement Randomization
1. Create multiple spawn points for objects
2. Randomize:
   - Position and orientation of objects
   - Scale within reasonable bounds
   - Number of objects in the scene

### Background Randomization
1. Create multiple background variations
2. Include different backgrounds (indoor/outdoor, different rooms)
3. Vary the complexity of backgrounds

## Step 6: Scripting Data Collection
### Basic Data Collection Script
Isaac Sim allows you to script the data collection process:

1. Create a Python script that controls:
   - Robot movement through the environment
   - Object placement and randomization
   - Triggering data capture
   - Managing file naming and organization

2. Key functions to use:
   - `world.reset()` - Reset the world to initial state
   - `world.step()` - Run a single simulation step
   - `camera.get_rgb_image()` - Get RGB image data
   - `camera.get_depth_data()` - Get depth data
   - `camera.get_segmentation()` - Get segmentation data

### Sample Data Collection Workflow
```python
# Pseudocode for a simple data collection workflow
for episode in range(num_episodes):
    # Randomize environment
    randomize_lighting()
    randomize_object_positions()
    randomize_textures()
    
    # Move robot to random position
    move_robot_randomly()
    
    # Capture data from all sensors
    rgb_image = camera.get_rgb_image()
    depth_data = camera.get_depth_data()
    segmentation = camera.get_segmentation()
    
    # Save data with appropriate annotations
    save_data_with_annotations(episode, rgb_image, depth_data, segmentation)
```

## Step 7: Exporting and Organizing Data
### File Formats
Isaac Sim can export data in various formats:
- **Images**: PNG, JPEG, EXR formats
- **Annotations**: JSON, XML, TXT formats
- **3D Data**: PLY, PC2 formats
- **Video**: MP4, AVI formats

### Data Organization Structure
Organize your data in a structure suitable for your ML pipeline:
```
dataset/
├── images/
│   ├── rgb/
│   └── depth/
├── annotations/
│   ├── bbox2d/
│   ├── segmentation/
│   └── oriented_bbox/
└── metadata/
    └── scene_config.json
```

## Step 8: Optimizing Data Generation
### Performance Optimization
- Use efficient USD files and scene organization
- Optimize rendering quality vs. speed based on needs
- Use multiple GPU instances for faster data generation
- Parallelize data generation across multiple environments

### Quality Assurance
1. Regularly validate generated data:
   - Check for artifacts or rendering issues
   - Verify annotations align with images
   - Ensure diversity in generated samples
2. Compare synthetic data statistics to real-world data when available

## Step 9: Validation and Testing
### Testing Model Transfer
1. Train a model on synthetic data only
2. Test on real-world data to measure transfer effectiveness
3. Use this feedback to adjust domain randomization parameters

### Ablation Studies
1. Test the impact of different domain randomization elements
2. Determine which variations provide the most improvement
3. Balance diversity with computational cost

## Troubleshooting Common Issues
- **Slow generation**: Reduce scene complexity or rendering quality
- **Poor quality annotations**: Check semantic labels are correctly applied
- **Domain gap too large**: Adjust randomization parameters to match real-world statistics
- **Memory issues**: Process data in smaller batches

## Summary
You've now learned how to generate synthetic data in Isaac Sim, which is crucial for training AI models without requiring real-world data collection. These techniques allow for rapid generation of diverse, labeled datasets for computer vision and robotics applications. 

## Next Steps
With synthetic data generation skills, you're well-equipped to start building complete robotics applications using Isaac Sim. Consider exploring the perception and VSLAM modules next to learn how to process the data you're generating.