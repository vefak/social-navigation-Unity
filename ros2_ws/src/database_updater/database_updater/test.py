from neo4j import GraphDatabase

class Neo4jConnection:
    def __init__(self, uri, user, password):
        self.driver = GraphDatabase.driver(uri, auth=(user, password))

    def close(self):
        self.driver.close()

    def run_query(self, query, parameters=None):
        with self.driver.session() as session:
            if parameters:
                result = session.run(query, parameters)
            else:
                result = session.run(query)
            
            return list(result)  # Collect results into a list

def update_robot_position(robot_id, new_position):
    # Neo4j connection details
    neo4j_uri = "bolt://localhost:7687"
    neo4j_user = "neo4j"
    neo4j_password = "12345678"

    # Create a connection to the Neo4j database
    conn = Neo4jConnection(neo4j_uri, neo4j_user, neo4j_password)

    # Define the Cypher query to update the robot's current position
    cypher_query = """
    MATCH (r:Robot {id: $robot_id})
    SET r.current_position = $new_position
    RETURN r
    """

    # Run the query with parameters
    result = conn.run_query(cypher_query, parameters={"robot_id": robot_id, "new_position": new_position})

    # Print the updated robot node
    for record in result:
        print("Updated Robot:", record["r"])

    # Close the connection
    conn.close()

# Example usage
if __name__ == "__main__":
    robot_id = "robot1"  # The ID of the robot you want to update
    new_position = [-10, 5]  # New position to set for the robot
    update_robot_position(robot_id, new_position)
