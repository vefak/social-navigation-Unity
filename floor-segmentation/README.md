# Floor Segmentation

This repository implements a pipeline consisting of hexagonal grid construction, structural information extraction and path planning.

## Folder Structure

The implementations (sources and headers) are all contained in the root directory. The other user-relevant directories are the following:

* **input/** - This folder should contain the image of the floor plan.
* **output/** - Will contain the result plots after the script finishes executing.
* **json/** - Contains the configuration file (config.json) where the parameter values are defined. All saved .json files also go to this folder.

The remaining directories:

* **python/** - Contains the python script for database communication and various other scripts used for prototyping.
* **nlohmann/** - Contains header only library for json parsing.

## Project Files

As a first step, we recommend taking a look at **example.cpp**.

* **example.cpp** - Demonstrates how the pipeline is invoked from start to finish. Default entry point.
* **python/database.py** - Defines the Python interface for communicating with the graph database.
* **nlohmann/json.hpp** - Header-only library for json parsing.
* **database.h/cpp** - Implement the C++ graph database interface, which invokes **python/database.py** using Python-C++ communication (via pybind11).
* **geometry.h/cpp** - Implementations of relevant geometric problems.
* **graph_util.h/cpp/tpp** - Implement mostly structure extraction based on graph algorithms, such as Vornoi map construction, wall and room detection.
* **grid.h/cpp/tpp** - Implement the (hexagonal) grid and various ways to construct it.
* **neo4j_demo.cpp** - Script demonstrating that the C++ database interface works.
* **planning.h/cpp/tpp** - Implement various global planners and the relevant classes.
* **plot.h/cpp/cpp** - Custom plotting for the grid and various data (rooms, walls, Voronoi cells, paths, etc.)
* **potential.h/cpp** - Define the various potential fields used in the local planners.
* **robot.h/cpp** - Implement the robot types and useful queries like collision detection.
* **save.h/cpp** - Saves a cypher query which constructs a graph-database that consists of the specified elements.
* **social_force.h/cpp/tpp** - The various local planners (SFM, A*, Oriented A*) based on social-aware potential fields. Also implements robot dynamics, Runge-Kutta integration and Dubins path construction.
* **transforms.h/cpp** - Implement the various coordinate transforms in a modular fashion, supporting composition and inversion.
* **util.h** - Header file with a few helpful functions that are used throughout the code.

## Parameters

The parameter values are all specified in **json/config.json**. We give a brief explanation of each parameter.

| Field                         | Type             | Default     | Description |
|------------------------------|------------------|-------------|-------------|
| `fileNames.inputMapName`     | `string`         | `floor_plan_unity.png`           | Input image filename for map parsing. |
| `fileNames.outputMapName`    | `string`         | `hex_grid.png`           | Filename for the generated hex grid. |
| `fileNames.outputTrajName`   | `string`         | `trajectory.txt`          | Filename where trajectory data will be saved. |

| Field                         | Type             | Default     | Description |
|------------------------------|------------------|-------------|-------------|
| `gridCompute.tileSize`       | `integer`         | `4`           | Size of each tile in pixels. |
| `gridCompute.detectionThreshold` | `float`     | `0.9`       | Normalized area threshold for classifying hex as occupied. |
| `gridCompute.displayRuntime` | `boolean`        | `false`     | Display runtime while executing script. |
| `gridCompute.wallMap.jumpPenalty` | `float`   | `0.01`      | Penalty for starting a new wall in the wall segmentation algorithm. |
| `gridCompute.wallMap.limitWallSize` | `boolean`| `true`      | Enable wall size limitation. |
| `gridCompute.wallMap.maxWallSize` | `integer`   | `100`       | Maximum allowed wall size (if limit is enabled). |

| Field                         | Type             | Default     | Description |
|------------------------------|------------------|-------------|-------------|
| `robot.length`               | `float`         | `2.5`           | Length of the robot (in meters). |
| `robot.width`                | `float`         | `0.9627`           | Width of the robot (in meters). |
| `robot.start.position._x` | `int` | `188` | Pixel x-coordinate of start. Listed only for reference. |
| `robot.start.position._y` | `int` | `360` | Pixel y-coordinate of start. Listed only for reference. |
| `robot.start.position.x`     | `float`         | `-28.95`           | x-coordinate in meters (Unity). |
| `robot.start.position.y`     | `float`         | `-0.0375`           | y-coordinate in meters (Unity). |
| `robot.start.theta`          | `float`         | `1.5708`         | Starting orientation in radians (Unity). |
| `robot.mass`                | `float`         | `70`           | Mass of the robot in kg. |
| `robot.maxVelocity` | `float` | `4` | Maximum robot velocity in m/s. |
| `robot.cruiseVelocity` | `float` | `1` | Cruise/normal operating velocity in m/s. |
| `robot.steeringAngle` | `float` | `1.135` | Max steering angle in radians. |
| `robot.wheelbase` | `float` | `2.0` | Distance between front and rear axles. |
| `robot.approximateNearestObstacle` | `boolean` | `true`      | Use fast approximation for nearest obstacle check. |
| `robot.circles[].x`          | `float`         | —           | Length-normalized x-position of a collision circle in local frame. |
| `robot.circles[].y`          | `float`         | —           | Length-normalized y-position of a collision circle in local frame. |
| `robot.circles[].radius`     | `float`         | —           | Length-normalized radius of collision circle. |
| `human.collisionRadius` | `float` | `0.5` | Collision radius for human beings. |
| `human.contractionCutoff` | `float` | `0.2` | Length contraction cutoff. **(\*)** |

| Field                         | Type             | Default     | Description |
|------------------------------|------------------|-------------|-------------|
| `pathPlanning.algorithm` | `string` | `Oriented-A-Star` | Chosen path planning algorithm. Options: `A-Star`, `Theta-Star`, `Oriented-A-Star`. |
| `pathPlanning.goal.position._x` | `int` | `940` | Pixel x-coordinate of goal. Listed only for reference. |
| `pathPlanning.goal.position._y` | `int` | `360` | Pixel y-coordinate of goal. Listed only for reference. |
| `pathPlanning.goal.position.x` | `float` | `-0.75` | Goal x-position in meters. |
| `pathPlanning.goal.position.y` | `float` | `-0.0375` | Goal y-position in meters. |
| `pathPlanning.goal.theta` | `float` | `-0.4292` | Goal orientation in radians. |
| `pathPlanning.nearestHexSanityCheck` | `boolean` | `true` | Print the nearest hex coordinates as a sanity check. |
| `pathPlanning.heuristicParams.heuristic` | `string` | `euclidean` | A* heuristic type. Options: `euclidean`, `hexagonal`. |
| `pathPlanning.heuristicParams.avoidWalls` | `boolean` | `true` | Penalize heuristic near walls. |
| `pathPlanning.heuristicParams.useValleyPotential` | `boolean` | `true` | Include valley potential that drives exploration towards the Voronoi skeleton. |
| `pathPlanning.heuristicParams.infinity` | `float` | `1e20` | Infinite cost. |
| `pathPlanning.heuristicParams.angleWeight` | `float` | `1.0` | Angular cost contribution in heuristic (A*). |
| `pathPlanning.plotSearchTree` | `boolean` | `false` | Plot the search tree for debugging. |
| `pathPlanning.OrientedAStarParams.angleCostMultiplier` | `float` | `1.0` | Multiplier for turning cost (A*). |
| `pathPlanning.OrientedAStarParams.angleSensitivity` | `float` | `10.0` | Sharpness of angle cost increase. |
| `pathPlanning.OrientedAStarParams.angleDeadzone` | `float` | `0.2` | Angle deadzone threshold. Cost is zero if angle is in deadzone. |
| `pathPlanning.OrientedAStarParams.turnratePenalty` | `float` | `1e4` | Penalty for exceeding maximum estimated turnrate. |
| `pathPlanning.OrientedAStarParams.angleCostFunction` | `string` | `exponential` | Function that maps angle to angular cost. Options: `linear`, `quadratic`, `exponential`. |

| Parameter Path | Type | Default/Value | Description |
|----------------|------|----------------|-------------|
| `localPlanner.planner` | `string` | `Oriented-A-Star` | Local planning algorithm. Options: `SFM`, `A-Star`, `Oriented-A-Star`. |
| `localPlanner.socialForce.linearParams.maxSamples` | `int` | `100000` | Maximum number of iterations of local planner. |
| `localPlanner.socialForce.linearParams.lookahead` | `int` | `5` | Lookahead horizon for determining local heading. |
| `localPlanner.socialForce.linearParams.samplingPeriod` | `float` | `0.1` | Sampling time interval in seconds. |
| `localPlanner.socialForce.linearParams.goalCutoffDistance` | `float` | `0.05` | Cutoff to consider goal reached. |
| `localPlanner.socialForce.influenceAuxiliaryAgents` | `boolean` | `false` | Allow influence of robot on auxiliary agents. |
| `localPlanner.socialForce.freeWalkParams.walkLength` | `int` | `50` | Length of free walk (one invocation of local planner). |
| `localPlanner.socialForce.freeWalkParams.skipSteps` | `int` | `2` | Number of starting steps to skip in free walk. |
| `localPlanner.socialForce.freeWalkParams.projectStartingPoint` | `boolean` | `false` | Whether to project current state onto the old trajectory. |
| `localPlanner.socialForce.rotationParams.alpha` | `float` | `3` | For tuning (real) poles of system. **(\*\*)**  |
| `localPlanner.socialForce.rotationParams.k_d` | `float` | `50` | Damping constant for sideward motion. **(\*\*)** |
| `localPlanner.socialForce.rotationParams.k_o` | `float` | `1` | Orthogonal force gain. **(\*\*)** |
| `localPlanner.socialForce.rotationParams.k_lambda` | `float` | `8` | Used to tune dominant time constant of system. **(\*\*)** |
| `localPlanner.gridParams.halfHeight`          | `int`     | `21`          | Half-height of local planning grid.       |
| `localPlanner.gridParams.halfWidth`           | `int`     | `21`          | Half-width of local planning grid.        |
| `localPlanner.gridParams.adaptive`            | `boolean` | `true`        | Enable adaptive grid sizing.              |
| `localPlanner.gridParams.gridDownscaleFactor` | `float`   | `0.05`        | Downscaling factor for local grid resolution w.r.t. global grid.   |
| `localPlanner.gridParams.centerSlide`         | `float`   | `0.5`         | Slide factor for recentralizing the grid towards the direction of motion. |
| `localPlanner.heuristicParams.heuristic`   | `string`  | `euclidean`   | Type of heuristic used in A\*-style local planner. Options: `euclidean`, `hexagonal`.                |
| `localPlanner.heuristicParams.avoidWalls`  | `boolean` | `false`       | Penalize heuristic near walls.          |
| `localPlanner.heuristicParams.infinity`    | `float`   | `1e20`        | Cost for unreachable nodes.             |
| `localPlanner.heuristicParams.angleWeight` | `float`   | `0.0`         | Weighting of angular cost in heuristic. |

| Parameter Path | Type | Default/Value | Description |
|----------------|------|----------------|-------------|
| `localPlanner.AStarParams.lightweight`              | `boolean` | `false`       | Use lightweight version of A\*.            |
| `localPlanner.AStarParams.downscaleField`           | `int`     | `10000`       | Potential field cost downscaling factor.                  |
| `localPlanner.AStarParams.lookahead`                | `int`     | `10`          | Find the local planner goal point by looking towards the future on the global path. Lookahead controls how far ahead we look.              |
| `localPlanner.AStarParams.fixedWalkLength`          | `boolean` | `true`        | Fix the walk length for each free walk.         |
| `localPlanner.AStarParams.rimTermination`           | `boolean` | `false`       | Stop when rim of planning area is reached. `NOTE`: Avoid using if possible, can lead to cycling behavior. |
| `localPlanner.AStarParams.stoppageDistance`         | `float`   | `0.5`         | Distance from dynamic obstacle at which robot should stop moving.  |
| `localPlanner.AStarParams.pathSmoothing.algorithm`  | `string`  | `None`        | Path smoothing (filtering) algorithm type. Options: `None`, `Gaussian`.            |
| `localPlanner.AStarParams.pathSmoothing.halfWindow` | `int`     | `7`           | Window size for smoothing.                 |
| `localPlanner.AStarParams.pathSmoothing.sigma`      | `float`   | `1.0`         | Kernel width for smoothing.        |
| `localPlanner.OrientedAStarParams.angleCostMultiplier` | `float`  | `10.0`        | Multiplier for orientation cost.       |
| `localPlanner.OrientedAStarParams.angleSensitivity`    | `float`  | `5.0`         | How sensitive cost is to angle change. |
| `localPlanner.OrientedAStarParams.angleDeadzone`       | `float`  | `0.2`         | Angle tolerance before applying cost.  |
| `localPlanner.OrientedAStarParams.angleCostFunction`   | `string` | `exponential` | Function mapping angle to angular cost.        |

| Parameter Path | Type | Default/Value | Description |
|----------------|------|----------------|-------------|
| `potentialField.precomputePotential` | `boolean` | `true`        | If true, potential field is computed at grid points and interpolated inbetween them. Faster than computing potential on the fly. |
| `potentialField.mapPotential.type`                                   | `string` | `VoronoiFieldPotential` | Type of static map potential. Options: `VoronoiFieldPotential`, `ObstacleRepulsionPotential`.        |
| `potentialField.mapPotential.voronoiFieldPotentialParams.scale`      | `float`  | `500.0`                 | Average scale for potential values. |
| `potentialField.mapPotential.voronoiFieldPotentialParams.alpha`      | `float`  | `10.0`                  | Decay rate of potential from walls. **(\*\*\*)**  |
| `potentialField.mapPotential.voronoiFieldPotentialParams.distObsMax` | `float`  | `3.0`                   | Beyond this distance, the potential becomes zero. **(\*\*\*)** |
| `potentialField.drivingPotential.type`                                        | `string` | `LookaheadPotential` | Driving potential function type. Options: `LookaheadPotential`, `GoalAndPathPotential`,`WindowPotential`.    |
| `potentialField.drivingPotential.distFunction`                                | `string` | `Conic`              | Distance function for potential shaping. Options: `Conic`, `Quadratic`. |
| `potentialField.drivingPotential.waypointAttraction`                          | `float`  | `10`                 | Attraction to intermediate waypoints.    |
| `potentialField.drivingPotential.goalAndPathPotentialParams.valleyAttraction` | `float`  | `0.75`               | Strength of attraction towards the Voronoi valley.    |
| `potentialField.drivingPotential.lookaheadPotentialParams.windowSize`         | `int`    | `10`                 | Window size for lookahead potential.          |
| `potentialField.drivingPotential.lookaheadPotentialParams.lookahead`          | `int`    | `5`                  | Similar to lookahead in local A*, but for orienting driving potential.               |
| `potentialField.drivingPotential.windowPotentialParams.windowSize`            | `int`    | `5`                  | Window size for window potential.     |
| `potentialField.drivingPotential.windowPotentialParams.cutoffDist`            | `float`  | `0.05`               | Distance cutoff for determining if point has reached current window.           |
| `potentialField.dynamicPotential.type`   | `string` | `EggshellPotential` | Dynamic potential which is expanded towards the direction of movement and contracted in the opposite direction. Options: `EggshellPotential`, `RipplePotential`. |
| `potentialField.dynamicPotential.cutoff` | `float`  | `0.2`               | Cutoff for length contraction.    |
| `potentialField.obstacleRepulsionParams.A` | `float` | `2500`        | Scale of repulsive potential. Height of the peak.     |
| `potentialField.obstacleRepulsionParams.B` | `float` | `1`           | Repulsion decay factor. How sharp is the peak. |

| Parameter Path       | Type      | Default/Value | Description                                 |
| -------------------- | --------- | ------------- | ------------------------------------------- |
| `rescaling.inMeters` | `boolean` | `true`        | Indicates if the input units are in meters of pixels. |
| `rescaling.xMin`     | `float`   | `-36`         | Minimum x value of map in Unity.              |
| `rescaling.xMax`     | `float`   | `12`          | Maximum x value of map in Unity.              |
| `rescaling.yMin`     | `float`   | `-13.5`       | Minimum y value of map in Unity.            |
| `rescaling.yMax`     | `float`   | `13.5`        | Maximum y value of map in Unity.             |


| Parameter Path      | Type    | Default/Value | Description                               |
| ------------------- | ------- | ------------- | ----------------------------------------- |
| `plot.defaultRGB.R` | `float` | `0.7`         | Default red channel value for plotting.          |
| `plot.defaultRGB.G` | `float` | `0.7`         | Default green channel value for plotting.        |
| `plot.defaultRGB.B` | `float` | `0.7`         | Default blue channel value for plotting.         |
| `plot.upscale`      | `int`   | `4`           | Upscaling factor for resolution of plot. |
| `plot.potentialFieldSnapshots.save`            | `boolean` | `true`        | Save potential field snapshots.          |
| `plot.potentialFieldSnapshots.name`            | `string`  | `potential`   | Filename prefix for snapshots.        |
| `plot.potentialFieldSnapshots.extension`       | `string`  | `png`         | File format for snapshots.            |
| `plot.potentialFieldSnapshots.stride`          | `float`   | `0.5`         | Time step interval between snapshots. |
| `plot.potentialFieldSnapshots.logPlot`         | `boolean` | `false`       | Use logarithmic scale for potential.  |
| `plot.potentialFieldSnapshots.flipColorScheme` | `boolean` | `true`        | Flip the color intensity of the plot (color = 1 - color).  |


| Parameter Path      | Type      | Default/Value | Description                      |
| ------------------- | --------- | ------------- | -------------------------------- |
| `json.databasePath` | `string`  | `../json`     | Directory for saving JSON data.  |
| `json.spacing`      | `int`     | `-1`          | Spacing used in JSON formatting. Default is -1 for no spacing: fastest but not very readable. |
| `json.save`         | `boolean` | `true`        | Save the generated JSON file.    |

| Parameter Path                        | Type     | Default/Value                            | Description                              |
| ------------------------------------- | -------- | ---------------------------------------- | ---------------------------------------- |
| `neo4j.uri`                           | `string` | `bolt://localhost:7687`                  | URI for Neo4j database connection.       |
| `neo4j.user`                          | `string` | `neo4j`                                  | Username for Neo4j connection.           |
| `neo4j.password`                      | `string` | `flame-chemist-vienna-nova-student-5397` | Password for Neo4j connection.           |
| `neo4j.queryParams.distanceThreshold` | `int`    | `30`                                     | Maximum distance at which nearby obstacle is returned by query. |





**(\*)** **Length contraction** - reduction in length in directions similar to the direction of movement, thereby resulting in a higher potential in front of the moving agent. Also used in collision detection.  (*)

**(\*\*)** See "**Walking Ahead: The Headed Social Force Model**" by Farina et al.

**(\*\*\*)** See "**Practical Search Techniques in Path Planning for Autonomous Driving**" by Dolgov et al.

## Algorithmic modes

For clarity of presentation, we list the configurable algorithm choices when launching the pipeline.

| Parameter Path                                       | Default Value           | Allowed Options                                                 |
| ---------------------------------------------------- | ----------------------- | --------------------------------------------------------------- |
| `pathPlanning.algorithm`                             | `Oriented-A-Star`       | `A-Star`, `Theta-Star`, `Oriented-A-Star`                       |
| `pathPlanning.heuristicParams.heuristic`             | `euclidean`             | `euclidean`, `hexagonal`                                        |
| `pathPlanning.OrientedAStarParams.angleCostFunction` | `exponential`           | `linear`, `quadratic`, `exponential`                            |
| `localPlanner.planner`                               | `Oriented-A-Star`       | `SFM`, `A-Star`, `Oriented-A-Star`                              |
| `localPlanner.heuristicParams.heuristic`             | `euclidean`             | `euclidean`, `hexagonal`                                        |
| `localPlanner.OrientedAStarParams.angleCostFunction` | `exponential`           | `linear`, `quadratic`, `exponential`                            |
| `localPlanner.AStarParams.pathSmoothing.algorithm`   | `None`                  | `None`, `Gaussian`                                              |
| `potentialField.mapPotential.type`                   | `VoronoiFieldPotential` | `VoronoiFieldPotential`, `ObstacleRepulsionPotential`           |
| `potentialField.drivingPotential.type`               | `LookaheadPotential`    | `LookaheadPotential`, `GoalAndPathPotential`, `WindowPotential` |
| `potentialField.drivingPotential.distFunction`       | `Conic`                 | `Conic`, `Quadratic`                                            |
| `potentialField.dynamicPotential.type`               | `EggshellPotential`     | `EggshellPotential`, `RipplePotential`                          |