This package implements a complete ball collection pipeline for TurtleBot3 robots, including vision-based detection, clustering, path planning, and navigation.

## System Architecture
The project consists of several key nodes working together:

### 1. Ball Detection Layer
- BallDetectorNode : Receives simulated ball positions from spawned_ball_positions topic
- YOLO Detector (from turtlebot3_vision package): Provides real-time ball detection
- Publishes detected ball positions as ball_positions (PointCloud2) and visual markers
### 2. Density Mapping Layer
- DensityMapBuilderNode :
  - Subscribes to detected_balls from YOLO detector
  - Creates an occupancy grid where each cell's value represents ball density
  - Increments density values for cells containing balls (max 100)
  - Publishes density_map (OccupancyGrid) and density_markers for visualization
### 3. Path Planning Layer
- SemanticPathPlannerNode :
  
  - Subscribes to density_map
  - Identifies high-density areas (threshold >50)
  - Performs K-means clustering to find ball clusters
  - Uses A* algorithm to plan paths to cluster centroids
  - Publishes semantic_path and sends navigation goals
- TspPlannerNode :
  
  - Subscribes to centroids from clustering
  - Solves Traveling Salesman Problem using greedy algorithm
  - Publishes optimal collection_path
### 4. Navigation Execution Layer
- PathPlannerNode :
  - Subscribes to collection_path
  - Sends navigation goals to Nav2 action server
  - Handles goal completion feedback
## Data Flow
1. Ball Detection → ball_positions → Density Mapping
2. Density Map → density_map → Semantic Path Planning
3. Clustered Centroids → centroids → TSP Planning
4. Optimal Path → collection_path → Navigation Execution
