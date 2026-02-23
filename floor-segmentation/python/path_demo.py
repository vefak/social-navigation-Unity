from neo4j import GraphDatabase
from time import time

class Neo4jConnection:
    def __init__(self, uri, user, password):
        self._driver = GraphDatabase.driver(uri, auth=(user, password))

    def close(self):
        self._driver.close()

    def run_query(self, query, parameters=None):
        with self._driver.session() as session:
            result = session.run(query, parameters)
            return [record for record in result]  # Result is consumed

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# URI examples: "neo4j://localhost", "neo4j+s://xxx.databases.neo4j.io"
connection_start = time()
uri = "neo4j://localhost:7687"
user = "neo4j"
password = "flame-chemist-vienna-nova-student-5397"
conn = Neo4jConnection(uri, user, password)
connection_finish = time()

print(f'Connection took {connection_finish - connection_start} [s]')

# Find shortest path over nodes
print('Finding shortest path...')
shortest_path_start = time()

# Convert node potentials into edge weights and preserve only valid edges
edge_weight_query = f"""MATCH (u:Node)-[r:NEIGHBORING]->(v:Node)
WHERE u.free=true AND v.free=true
MERGE (u)-[f:FREELY_CONNECTED]->(v)
SET f.weight=v.potential + 1
"""

result = conn.run_query(edge_weight_query)

# Shortest path query
startNodeIdx = 11036
goalNodeIdx = 10992

shortest_path_query = f"""MATCH (u:Node)
    WHERE u.rawIdx = {startNodeIdx}
    MATCH (v:Node)
    WHERE v.rawIdx = {goalNodeIdx}
    CALL apoc.algo.dijkstra(u, v, 'FREELY_CONNECTED', 'weight') YIELD path, weight
    RETURN path, weight"""

result = conn.run_query(shortest_path_query)
shortest_path_end = time()

path = [Point(edge._end_node['x'], edge._end_node['y']) for edge in result[0]['path']]
path = [Point(result[0]['path'].start_node['x'], result[0]['path'].start_node['y'])] + path  # Add first node

with open('./output/shortest_path_neo4j.txt', 'w') as f:
    for node in path:
        f.write(f'{node.x} {node.y}\n')


# Query for shortest path between rooms
print('Finding shortest path through room graph...')
room_path_start = time()

# Find room containing start and goal
def room_id(node_id):
    room_id_query = f"""MATCH (r: Room)-[:CONTAINS]->(n:Node)
    WHERE n.rawIdx={node_id}
    RETURN r.index
    """
    result = conn.run_query(room_id_query)
    return result[0][0]


room_start_index = room_id(startNodeIdx)
room_goal_index = room_id(goalNodeIdx)

room_path_query = f"""MATCH (r1: Room)
WHERE r1.index = {room_start_index}
MATCH (r2: Room)
WHERE r2.index = {room_goal_index}
CALL apoc.algo.dijkstra(r1, r2, 'NEIGHBORING', 'distance') YIELD path, weight
RETURN path, weight"""

result = conn.run_query(room_path_query)

room_path = [(edge._end_node['index'], edge._start_node['index'], edge['distance']) for edge in result[0]['path']]
print(room_path)

room_path_end = time()

# Elapsed time: 
print(f'Elapsed time: {room_path_end - room_path_start} [s]')
