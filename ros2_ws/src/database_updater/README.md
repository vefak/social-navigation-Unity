## database_updater

This package has been developed to facilitate seamless communication between Neo4j and ROS2, allowing for real-time data exchange and integration. It enables ROS2 nodes to interact with a Neo4j database, supporting tasks such as storing, retrieving, and updating spatial data related to robots and surrounding entities. This setup is particularly useful in applications requiring persistent data storage and relational querying capabilities for dynamic robot environments.


- **neo4j_node.py:** This ROS2 node, `neo4j_node`, integrates ROS2 and Neo4j to synchronize robot and human positions, rotations, and velocities with a Neo4j database. It connects to Neo4j, sets up initial database entries for a robot, multiple humans, and obstacles, and subscribes to specific ROS2 topics for position updates. Each update dynamically modifies the Neo4j database with new positions and velocities, providing real-time data storage and retrieval. This setup is ideal for applications that need spatial data persistence and relational querying in a Neo4j environment.

#### Neo4j Database Setup
Install Neo4j and ensure it is running on your machine.
Update Neo4j connection details in the script:
```bash 
URI: bolt://localhost:7687
User: neo4j
Password: 12345678 
```
#### Key Functions
* **run_neo4j_query:** Executes a Neo4j query with optional parameters.
* **create_query:** Creates nodes for the robot and humans with initial positions and velocities.
* **create_near_query:** Creates obstacle nodes and groups nearby obstacles.
* **robot_callback:** Updates robot position, rotation, and velocity in the database based on incoming data.
* **human_position_callback:** Updates human entities similarly.

* The node listens on the following topics:
  * `/robot`: Robot position and velocity data.
  * `/human1`, `/human2`, `/human3`, `/human4`: Human position and velocity data.


- **test.py & dynamic_object_subscriber.py** : These two codes and this study's different components were tested separately and later combined into a single node.
