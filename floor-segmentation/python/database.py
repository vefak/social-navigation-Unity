from neo4j import GraphDatabase
import json
import os
from dataclasses import dataclass
from math import pi
import time


class Neo4jConnection:
    def __init__(self, uri, user, password):
        self._driver = GraphDatabase.driver(uri, auth=(user, password))

    def close(self):
        self._driver.close()

    def run_query(self, query, parameters=None):
        with self._driver.session() as session:
            result = session.run(query, parameters)
            return [record for record in result]  # Result is consumed


@dataclass
class QueryParams:
    distance_threshold: float


@dataclass
class Robot:
    position: list
    rotation: list
    linear_velocity: list
    angular_velocity: list


class Neo4jInterface:
    def __init__(self, root):
        with open(f'{root}/json/config.json') as json_data:
            data = json.load(json_data)
            json_data.close()
            uri = data['neo4j']['uri']
            user = data['neo4j']['user']
            password = data['neo4j']['password']
            distance_threshold = data['neo4j']['queryParams']['distanceThreshold']

        self.conn = Neo4jConnection(uri, user, password)
        self.query_params = QueryParams(distance_threshold=distance_threshold)

    def close(self):
        self.conn.close()

    def demo_setup(self):
        # Setup robot and humans in the environment
        query = """
        MERGE (r1:Robot {id: 'robot'})
        SET r1.position=[0,0,1], r1.rotation=[0,0,0], r1.linear_velocity=[0,0,0], r1.angular_velocity=[0,0,0]
        MERGE (h1:Human {id: 'human1'})
        SET h1.position=[1,0,0], h1.rotation=[0,0,0], h1.linear_velocity=[0,0,0], h1.angular_velocity=[0,0,0]
        MERGE (h2:Human {id: 'human2'})
        SET h2.position=[0,0,3.5], h2.rotation=[0,0,0], h2.linear_velocity=[0,0,0], h2.angular_velocity=[0,0,0]
        MERGE (h3:Human {id: 'human3'})
        SET h3.position=[0,0,-8], h3.rotation=[0,0,0], h3.linear_velocity=[0,0,0], h3.angular_velocity=[0,0,0]
        MERGE (h4:Human {id: 'human4'})
        SET h4.position=[3,0,-3], h4.rotation=[0,0,0], h4.linear_velocity=[0,0,0], h4.angular_velocity=[0,0,0]
        """
        self.conn.run_query(query)

    # Nearby dynamic obstacles around p
    def get_dynamic_obstacles(self, x, y):
        p = (x, y)

        query = f"""
        WITH point({{x: {p[0]}, y: {p[1]}}}) AS robotLocation
        MATCH (h: Human)
        WITH h, robotLocation, point({{x: h.position[0], y: h.position[2]}}) AS humanLocation
        WHERE point.distance(robotLocation, humanLocation) <= {self.query_params.distance_threshold}
        RETURN h as human
        """

        result = self.conn.run_query(query)
        entries = [entry['human'] for entry in result]
        obstacles = [Robot(entry['position'], entry['rotation'], entry['linear_velocity'],
                           entry['angular_velocity']) for entry in entries]
        return obstacles

    def get_robot_state(self):
        query = """
        MATCH (r: Robot {id : 'robot'})
        RETURN r as robot
        """
        robot_neo4j = self.conn.run_query(query)[0]['robot']

        robot = Robot(position=robot_neo4j['position'], rotation=robot_neo4j['rotation'],
                      linear_velocity=robot_neo4j['linear_velocity'],
                      angular_velocity=robot_neo4j['angular_velocity'])

        return robot

    def get_robot_position(self):
        pos3d = self.get_robot_state().position
        return (pos3d[0], pos3d[2])  # Unity coordinates

    def get_robot_orientation(self):
        return self.get_robot_state().rotation[1]  # y-axis is up in Unity

    def get_robot_velocity(self):
        vel3d = self.get_robot_state().linear_velocity
        return (vel3d[0], vel3d[2])  # Unity coordinates

    def get_robot_angular_velocity(self):
        # y-axis is up in Unity
        return self.get_robot_state().angular_velocity[1]

    def update_robot_position(self, x: float, y: float):
        position = (x, y)
        query = f"""
        MATCH (r: Robot {{id : 'robot'}})
        SET r.position = [{position[0]}, 0, {position[1]}]
        """
        self.conn.run_query(query)

    def update_robot_orientation(self, rotation: float):
        query = f"""
        MATCH (r: Robot {{id : 'robot'}})
        SET r.rotation = [0, {rotation}, 0]
        """
        self.conn.run_query(query)

    def update_robot_velocity(self, v_x: float, v_y: float):
        linear_velocity = (v_x, v_y)
        query = f"""
        MATCH (r: Robot {{id : 'robot'}})
        SET r.linear_velocity = [{linear_velocity[0]}, 0, {linear_velocity[1]}]
        """
        self.conn.run_query(query)

    def update_robot_angular_velocity(self, angular_velocity: float):
        query = f"""
        MATCH (r: Robot {{id : 'robot'}})
        SET r.angular_velocity = [0, {angular_velocity}, 0]
        """
        self.conn.run_query(query)


def print_time(flag_0, flag_1):
    print(f'Done. Elapsed {flag_1 - flag_0} [s].\n')


def main():
    root = '.'  # Root directory of the FloorSegmentation project

    print('Initializing interface...')
    flag_0 = time.time()
    db = Neo4jInterface(root)
    print_time(flag_0, time.time())

    # Set-up a basic example
    print('Example setup...')
    flag_1 = time.time()
    db.demo_setup()
    print_time(flag_1, time.time())

    # Check that localization queries work
    print('Localization queries...')
    flag_2 = time.time()
    print(db.get_robot_state())
    print('position: ', db.get_robot_position())
    print('orientation: ', db.get_robot_orientation())
    print('linear velocity: ', db.get_robot_velocity())
    print('angular velocity: ', db.get_robot_angular_velocity())
    print_time(flag_2, time.time())

    # Check that nearby obstacle query works
    print('Dynamic obstacles query...')
    flag_3 = time.time()
    x, y = db.get_robot_position()
    print('dynamic obstacles: ', db.get_dynamic_obstacles(x, y))
    print_time(flag_3, time.time())

    # Check that update queries work
    print('Update queries...')
    flag_4 = time.time()

    db.update_robot_position(-1, 1)
    db.update_robot_orientation(pi / 2)
    db.update_robot_velocity(0, 1)
    db.update_robot_angular_velocity(1)

    print(db.get_robot_state())
    print('position: ', db.get_robot_position())
    print('orientation: ', db.get_robot_orientation())
    print('linear velocity: ', db.get_robot_velocity())
    print('angular velocity: ', db.get_robot_angular_velocity())

    print_time(flag_4, time.time())

    # Check again nearby obstacles
    print('Dynamic query for updated robot position...')
    flag_5 = time.time()
    x, y = db.get_robot_position()
    print('dynamic obstacles: ', db.get_dynamic_obstacles(x, y))
    print_time(flag_5, time.time())

    db.close()




if __name__ == '__main__':
    main()
