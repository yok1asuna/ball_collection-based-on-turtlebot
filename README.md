This package implements a complete ball collection pipeline for TurtleBot3 robots, including vision-based detection, clustering, path planning, and navigation.

## System Architecture

The project consists of several key nodes working together:

### 1. Ball Detection Layer

- BallDetectorNode : Receives  ball positions from yolo\_d_etector_\_node
- YOLO Detector  provides real-time ball detection
- Publishes detected ball positions as ball\_positions (PointCloud2) and visual markers

### 2. Density Mapping Layer

- DensityMapBuilderNode :
  - Subscribes to detected\_balls from YOLO detector
  - Creates an occupancy grid where each cell's value represents ball density
  - Increments density values for cells containing balls (max 100)
  - Publishes density\_map (OccupancyGrid) and density\_markers for visualization

### 3. Path Planning Layer

- SemanticPathPlannerNode :
  - Subscribes to density\_map
  - Identifies high-density areas (threshold >50)
  - Performs K-means clustering to find ball clusters
  - Uses A\* algorithm to plan paths to cluster centroids
  - Publishes semantic\_path and sends navigation goals
- TspPlannerNode :
  - Subscribes to centroids from clustering
  - Solves Traveling Salesman Problem using greedy algorithm
  - Publishes optimal collection\_path

### 4. Navigation Execution Layer

- PathPlannerNode :
  - Subscribes to collection\_path
  - Sends navigation goals to Nav2 action server
  - Handles goal completion feedback

## Data Flow

1. Ball Detection → ball\_positions → Density Mapping
2. Density Map → density\_map → Semantic Path Planning
3. Clustered Centroids → centroids → TSP Planning
4. Optimal Path → collection\_path → Navigation Execution

