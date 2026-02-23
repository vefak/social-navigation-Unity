import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from unitycustommsg.msg import TwistTransformUnity

from neo4j import GraphDatabase  # Neo4j driver


class Neo4jROS2Subscriber(Node):

    def __init__(self, uri, user, password):
        super().__init__("neo4j_node")  # Initialize the ROS2 Node

        # Neo4j connection
        self.driver = GraphDatabase.driver(uri, auth=(user, password))

        # Retrieve and print the connected database name
        database_name = self.get_database_name()
        if database_name:
            # print(f"Connected to database: {database_name}")
            self.get_logger().info(f"Connected to database: {database_name}")
        else:
            # print("Could not retrieve the database name")
            self.get_logger().warn("Could not retrieve the database name")

        # Delete everything in database
        # self.delete_database(),
        # Delete human robot nodes
        self.delete_human_robot_nodes()
        self.delete_group_nodes()

        # ROS2 subscription to a topic
        self.subscription = self.create_subscription(
            TwistTransformUnity,
            "/robot",  # Replace this with the topic name you're subscribing to
            lambda msg: self.robot_callback(msg, "/robot"),
            10,  # QoS profile depth
        )

        # Create subscriptions for humans
        # --- Auto-detect human topics ---
        self.human_topics = []
        all_topics = self.get_topic_names_and_types()
        for name, types in all_topics:
            if (
                name.startswith("/dynamic_")
                and "unitycustommsg/msg/TwistTransformUnity" in types
            ):
                self.human_topics.append(name)
                self.create_subscription(
                    TwistTransformUnity,
                    name,
                    lambda msg, t=name: self.human_position_callback(msg, t),
                    10,
                )

        # Create Query in database
        self.create_query()
        # Create Near by Obstacle
        self.get_logger().info("Node initialized and ready to receive messages.")

    def run_neo4j_query(self, query, parameters=None):
        with self.driver.session() as session:
            if parameters:
                result = session.run(query, parameters)
            else:
                result = session.run(query)
            return result

    def delete_human_robot_nodes(self):
        query = """
                MATCH (n)
                WHERE n:Robot OR n:Human
                DETACH DELETE n
                """
        self.run_neo4j_query(query)

    def delete_group_nodes(self):
        query = """
        MATCH (n)
        WHERE n:Group
        DETACH DELETE n
        """
        self.run_neo4j_query(query)

    def delete_database(self):
        self.run_neo4j_query("""MATCH (n) DETACH DELETE n""")

    def create_query(self):
        robot_query = """
        CREATE (r1:Robot {id: 'robot', position: [0,0,0], rotation: [0, 0, 0],
                            linear_velocity: [0,0,0], angular_velocity: [0,0,0]})
        """
        self.run_neo4j_query(robot_query)

        # Add all humans dynamically
        for topic in self.human_topics:
            human_id = topic.lstrip("/")
            human_query = f"""
            CREATE (h:Human {{id: '{human_id}', position: [0,0,0], rotation: [0, 0, 0],
                                linear_velocity: [0,0,0], angular_velocity: [0,0,0]}})
            """
            self.run_neo4j_query(human_query)

    def get_database_name(self):
        # Open a session and run the query
        with self.driver.session() as session:
            result = session.run("CALL db.info()")
            record = result.single()
            if record:
                return record["name"]
            else:
                return None

    def close(self):
        self.driver.close()

    def meters_to_pixels(self, x_m, y_m):
        """
        Converts world (meters) coordinates to image pixel coordinates.
        """
        img_width = 1280
        img_height = 720

        x_min = -36
        x_max = 12
        y_min = -13.5
        y_max = 13.5

        x_scale = x_max - x_min
        y_scale = y_max - y_min

        pixels_per_meter_x = img_width / x_scale
        pixels_per_meter_y = img_height / y_scale

        pixel_x = (x_m - x_min) * pixels_per_meter_x
        pixel_y = img_height - (y_m - y_min) * pixels_per_meter_y

        return int(pixel_x), int(pixel_y)

    def find_closest_hex(self, robot_id):

        query = """
        MATCH (r:Robot {id: $robot_id})
        OPTIONAL MATCH (r)-[rel:IS_AT]->()
        DELETE rel
        WITH r
        WITH r, point({x: r.pixel_position[0], y: r.pixel_position[1]}) AS robotPoint
        MATCH (h:HexCell)
        WITH r, h, robotPoint, point({x: h.x, y: h.y}) AS hexPoint
        WITH r, h, point.distance(robotPoint, hexPoint) AS dist
        ORDER BY dist ASC
        LIMIT 1
        WITH r, h
        OPTIONAL MATCH (room:Room)-[:CONTAINS_CELL]->(h)
        MERGE (r)-[:IS_AT]->(h)
        WITH r, h,room
        WHERE room IS NOT NULL
        MERGE (r)-[:IS_AT]->(room)        
        """

        with self.driver.session() as session:
            result = session.run(
                query,
                parameters={"robot_id": robot_id},
            )

    def robot_callback(self, msg, topic):

        # Remove the leading '/' from the topic name to get the entity ID
        robot_id = topic.lstrip("/")

        # Extract data
        translation = msg.transform.transform.translation
        rotation = msg.transform.transform.rotation
        twist = msg.twist

        pixel_x, pixel_y = self.meters_to_pixels(translation.x, translation.z)
        # print(f"Robot pixel coordinates: ({pixel_x}, {pixel_y})")

        # If you need to create a Neo4j query from the  data
        active_pos_query = """
        MATCH (r:Robot {id: $robot_id})
        SET r.position = $new_position
        SET r.rotation = $new_rotation
        SET r.linear_velocity = $new_linear_velocity
        SET r.angular_velocity = $new_angular_velocity
        SET r.pixel_position = $new_pixel_position
        RETURN r
        """
        new_position = [translation.x, translation.y, translation.z]
        new_rotation = [rotation.x, rotation.y, rotation.z]
        new_linear_velocity = [twist.linear.x, twist.linear.y, twist.linear.z]
        new_angular_velocity = [twist.angular.x, twist.angular.y, twist.angular.z]
        new_pixel_position = [pixel_x, pixel_y]

        self.run_neo4j_query(
            active_pos_query,
            parameters={
                "robot_id": robot_id,
                "new_position": new_position,
                "new_rotation": new_rotation,
                "new_linear_velocity": new_linear_velocity,
                "new_angular_velocity": new_angular_velocity,
                "new_pixel_position": new_pixel_position,
            },
        )

        self.find_closest_hex(robot_id)

    def human_position_callback(self, msg, topic):

        # Remove the leading '/' from the topic name to get the entity ID
        human_id = topic.lstrip("/")

        # Extract Twist and TransformStamped data from the custom message
        twist = msg.twist
        translation = msg.transform.transform.translation
        rotation = msg.transform.transform.rotation

        # Update human's position in Neo4j
        human_pos_query = """
            MATCH (h:Human {id: $human_id})
            SET h.position = $new_position
            SET h.rotation = $new_rotation
            SET h.linear_velocity = $new_linear_velocity
            SET h.angular_velocity = $new_angular_velocity
            SET h.pixel_position = $new_pixel_position
            RETURN h
            """
        new_position = [translation.x, translation.y, translation.z]
        new_rotation = [rotation.x, rotation.y, rotation.z]
        new_linear_velocity = [twist.linear.x, twist.linear.y, twist.linear.z]
        new_angular_velocity = [twist.angular.x, twist.angular.y, twist.angular.z]
        pixel_x, pixel_y = self.meters_to_pixels(translation.x, translation.z)
        new_pixel_position = [pixel_x, pixel_y]

        self.run_neo4j_query(
            human_pos_query,
            parameters={
                "human_id": human_id,
                "new_position": new_position,
                "new_rotation": new_rotation,
                "new_linear_velocity": new_linear_velocity,
                "new_angular_velocity": new_angular_velocity,
                "new_pixel_position": new_pixel_position,
            },
        )
        self.run_cypher_file(
            "./src/database_updater/database_updater/humangrouping.cypher"
        )

    def run_cypher_file(self, filepath):
        try:
            with open(filepath, "r") as file:
                cypher_statements = file.read()

            # Split by semicolon, remove empty parts
            statements = [
                stmt.strip() for stmt in cypher_statements.split(";") if stmt.strip()
            ]

            with self.driver.session() as session:
                for stmt in statements:
                    session.run(stmt)
                    # self.get_logger().info(f"Executed query:\n{stmt}")

        except Exception as e:
            self.get_logger().error(f"Error running .cypher file: {str(e)}")


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Neo4j connection details (update with your own)
    neo4j_uri = "bolt://localhost:7687"
    neo4j_user = "neo4j"
    neo4j_password = "12345678"

    # Create the node and spin it to keep it running
    neo4j_node = Neo4jROS2Subscriber(neo4j_uri, neo4j_user, neo4j_password)
    rclpy.spin(neo4j_node)

    # Shutdown and cleanup when done
    neo4j_node.close()
    neo4j_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
