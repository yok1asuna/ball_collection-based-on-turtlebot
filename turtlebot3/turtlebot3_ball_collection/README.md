# TurtleBot3 Ball Collection Package

This package implements an complete ball collection system for TurtleBot3 robots using  YOLOv11 neural network for real-time multi-target detection, combined with SLAM spatial coordinate projection to build a target density cost map. It employs a heuristic priority path planning algorithm based on non-uniform potential fields, incorporating K-Means density clustering to identify high-yield areas, and uses an improved A\* planner to guide the robot along potential field gradients for efficient collection of outliers.

## Features

- Detects balls using YOLO and converts 2D pixel coordinates to 3D world coordinates
- Builds a dynamic density map with Gaussian distribution and time decay
- Identifies high-density areas using K-means clustering
- Plans optimal paths using A\* with density-based cost function
- Optimizes cluster visitation order using TSP
- Navigates to high-density areas to maximize ball collection efficiency

## Architecture

1. **Detection Layer**: YOLOv11 processes RGB-D images to detect balls in 3D space
2. **Perception Layer**: Transforms detections to map coordinates and builds density cost map
3. **Planning Layer**: K-Means clustering + potential field A\* for optimal collection paths
4. **Execution Layer**: Nav2 integration for autonomous navigation and collection

##

## Topics



## Dependencies

- ROS2 Humble
- Nav2
- ultralytics (YOLOv11)
- OpenCV
- cv\_bridge

## Usage

1. Set environment variables:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```

<br />

<br />

1. Launch the system:
   ```bash
   ros2 launch turtlebot3_ball_collection full_system.launch.py
   ```
ros2 run teleop_twist_keyboard teleop_twist_keyboard

2. The robot will automatically detect balls using YOLO, build density maps, plan optimal collection paths, and navigate to targets.

## Algorithm Details

### Density Cost Map

- &#x20;rotate once after startup.

* Converts YOLO 3D detections to occupancy grid
* Higher density regions have lower costs
* Enables potential field-based path planning

### K-Means Clustering

- Groups high-density areas into clusters
- Identifies optimal collection centroids
- Reduces planning complexity

### Potential Field A\*

- Combines traditional A\* with potential field costs
- Prioritizes paths through high-density regions
- Generates smooth, efficient trajectories

## Performance

- **Detection**: Real-time multi-target detection with YOLOv11
- **Planning**: Efficient density-aware path optimization
- **Navigation**: Seamless Nav2 integration
- **Scalability**: Handles dynamic environments and multiple targets

