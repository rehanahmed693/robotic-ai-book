# Quickstart Guide: AI-Robot Brain (NVIDIA Isaac™)

## Prerequisites

Before starting with the AI-Robot Brain (NVIDIA Isaac™) project, ensure you have the following:

1. **NVIDIA GPU**: With compute capability 6.0 or higher
2. **Docker**: Install Docker Desktop with GPU support enabled
3. **Isaac Sim**: Download and install Isaac Sim 2023.1 or later from NVIDIA Developer website
4. **ROS 2**: Install Humble Hawksbill or later version
5. **Isaac ROS**: Install Isaac ROS packages compatible with your ROS 2 version
6. **Nav2**: Install Navigation2 packages for your ROS 2 version
7. **Python 3.8+**: Required for scripting and development

## Setting up Isaac Sim Environment

### Step 1: Launch Isaac Sim
```bash
# Navigate to Isaac Sim installation directory
cd /path/to/isaac-sim
./isaac-sim.sh
```

### Step 2: Create a New Simulation Environment
1. Open Isaac Sim
2. Create a new scene or load an existing one
3. Configure physics settings (gravity, friction, etc.)
4. Add your robot model to the environment

### Step 3: Configure Robot Model
1. Import or create a robot URDF model
2. Set up joint configurations and kinematic properties
3. Add necessary sensors to the robot (cameras, LIDAR, etc.)

## Setting up Isaac ROS Perception Pipeline

### Step 1: Install Isaac ROS Packages
```bash
# Using apt (Ubuntu)
sudo apt update
sudo apt install ros-humble-isaac-ros-perception
sudo apt install ros-humble-isaac-ros-slam

# Or using source build
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
# Build with colcon build
```

### Step 2: Launch VSLAM Pipeline
```bash
# Launch the visual SLAM pipeline
ros2 launch isaac_ros_visual_slam visual_slam.launch.py
```

This will start the VSLAM pipeline that processes visual data from Isaac Sim and creates a map of the environment.

## Setting up Nav2 Navigation

### Step 1: Install Nav2
```bash
# Using apt (Ubuntu)
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

# Or using source build
git clone https://github.com/ros-planning/navigation2.git
# Build with colcon build
```

### Step 2: Configure Nav2
1. Create a Nav2 configuration file for your robot
2. Set up costmap parameters for obstacle detection
3. Configure global and local planners

### Step 3: Launch Nav2
```bash
# Launch the full Nav2 stack
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    map:=/path/to/your/map.yaml
```

## Basic Navigation Example

### Step 1: Start Isaac Sim with a Robot
1. Load a scene with your robot in Isaac Sim
2. Ensure sensors are properly configured
3. Connect Isaac Sim to ROS 2

### Step 2: Run Perception Pipeline
```bash
# In a new terminal
ros2 launch isaac_ros_visual_slam visual_slam.launch.py
```

### Step 3: Send Navigation Goals
```bash
# In another terminal, send a navigation goal
ros2 run nav2_msgs navigate_to_pose -- \
    -x 1.0 \
    -y 2.0 \
    -z 0.0 \
    -w 1.0
```

## Running the Complete Pipeline

### Step 1: Launch Isaac Sim
```bash
./isaac-sim.sh
```

### Step 2: Launch Isaac ROS Perception
```bash
ros2 launch isaac_ros_visual_slam visual_slam.launch.py
```

### Step 3: Launch Nav2
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```

### Step 4: Send Navigation Commands
```bash
# Use RViz to send navigation goals
ros2 run rviz2 rviz2
```

## Troubleshooting

### Common Issues:

1. **GPU Not Detected**:
   - Verify that your NVIDIA drivers are up to date
   - Check that Docker is configured with GPU support using `nvidia-smi`

2. **Communication Issues**:
   - Ensure the same ROS_DOMAIN_ID is set in Isaac Sim and ROS 2
   - Check that both environments are using the same network configuration

3. **Performance Issues**:
   - Monitor GPU utilization using `nvidia-smi`
   - Reduce simulation complexity if performance is poor

4. **Map Quality Issues**:
   - Ensure sufficient lighting in the simulation environment
   - Verify that visual features are present in the environment
   - Check that camera parameters match reality

## Resources

- [NVIDIA Isaac Sim Documentation](https://docs.nvidia.com/isaac-sim/)
- [Isaac ROS GitHub Repository](https://github.com/NVIDIA-ISAAC-ROS)
- [Navigation2 Documentation](https://navigation.ros.org/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

This quickstart guide provides a basic setup for working with NVIDIA Isaac technologies. For more advanced usage and specific applications, refer to the detailed chapters in the main documentation.