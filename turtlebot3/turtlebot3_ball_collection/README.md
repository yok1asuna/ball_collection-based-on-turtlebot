# TurtleBot3 Ball Collection Package

This package implements an complete ball collection system for TurtleBot3 robots using  YOLOv11 neural network for real-time multi-target detection, combined with SLAM spatial coordinate projection to build a target density cost map. It employs a heuristic priority path planning algorithm based on non-uniform potential fields, incorporating K-Means density clustering to identify high-yield areas, and uses an improved A* planner to guide the robot along potential field gradients for efficient collection of outliers.

## Features

- **YOLOv11 Detection**: neural network for real-time multi-target ball detection
- **Density Cost Map**: Projects detection results onto local grid to build density-based cost map
- **K-Means Clustering**: Identifies high-density regions for prioritized collection
- **Potential Field Planning**: Non-uniform potential field algorithm with density weights
- **Improved A* Planning**: Enhanced A* with potential field integration for smooth trajectories
- **SLAM Integration**: Compatible with Nav2 navigation stack

## Architecture

1. **Detection Layer**: YOLOv11 processes RGB-D images to detect balls in 3D space
2. **Perception Layer**: Transforms detections to map coordinates and builds density cost map
3. **Planning Layer**: K-Means clustering + potential field A* for optimal collection paths
4. **Execution Layer**: Nav2 integration for autonomous navigation and collection

## Launch Options

### Full System
```bash
ros2 launch turtlebot3_ball_collection full_system.launch.py
```

### Ball Collection Pipeline Only
```bash
ros2 launch turtlebot3_ball_collection ball_collection.launch.py
```

## Parameters

- `resolution`: Density map resolution (default: 0.1m)
- `width`: Density map width (default: 100 cells)
- `height`: Density map height (default: 100 cells)
- `origin_x`: Map origin X (default: -5.0m)
- `origin_y`: Map origin Y (default: -5.0m)

## Topics

- `/detected_balls`: YOLO-detected ball positions (PointCloud2)
- `/density_map`: Target density cost map (OccupancyGrid)
- `/semantic_path`: Planned collection path (Path)
- `/density_markers`: Density visualization markers

## Dependencies

- ROS2 Humble
- Nav2
- ultralytics (YOLOv11)
- OpenCV
- cv_bridge

## Usage

1. Set environment variables:
   ```bash
   export TURTLEBOT3_MODEL=waffle
   ```

2. Install YOLO dependencies:
   ```bash
   pip install ultralytics
   ```

3. Launch the system:
   ```bash
   ros2 launch turtlebot3_ball_collection full_system.launch.py
   ```

4. The robot will automatically detect balls using YOLO, build density maps, plan optimal collection paths, and navigate to targets.

## Algorithm Details

### Density Cost Map
- Converts YOLO 3D detections to occupancy grid
- Higher density regions have lower costs (attractive)
- Enables potential field-based path planning

### K-Means Clustering
- Groups high-density areas into clusters
- Identifies optimal collection centroids
- Reduces planning complexity

### Potential Field A*
- Combines traditional A* with potential field costs
- Prioritizes paths through high-density regions
- Generates smooth, efficient trajectories

### Edge Computing
- Designed for lightweight deployment on edge platforms
- Real-time performance with YOLOv11 nano model
- Minimal computational overhead

## Performance

- **Detection**: Real-time multi-target detection with YOLOv11
- **Planning**: Efficient density-aware path optimization
- **Navigation**: Seamless Nav2 integration
- **Scalability**: Handles dynamic environments and multiple targets