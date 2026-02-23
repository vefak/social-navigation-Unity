from time import time

from neo4j import GraphDatabase


class Neo4jConnection:
    def __init__(self, uri, user, password):
        self._driver = GraphDatabase.driver(uri, auth=(user, password))

    def close(self):
        self._driver.close()

    def run_query(self, query, parameters=None):
        with self._driver.session() as session:
            result = session.run(query, parameters)
            return [record for record in result]  # Result is consumed


# URI examples: "neo4j://localhost", "neo4j+s://xxx.databases.neo4j.io"
uri = "bolt://localhost:7687"
user = "neo4j"
password = "12345678"
conn = Neo4jConnection(uri, user, password)

# Enter nodes into database
print("Inserting hex grid into the database...")
insert_nodes_start = time()

insert_nodes_query = """CALL apoc.load.json("database.json")
YIELD value
UNWIND value.hexGrid as hex
UNWIND range(0, size(hex.neighbors) - 1) as direction
WITH hex, hex.neighbors[direction] as neighborIdx, direction

MERGE (u:Node {rawIdx:hex.rawIdx})
SET u.col=hex.col, u.free=hex.free, u.row=hex.row, u.x=hex.x, u.y=hex.y, u.potential=hex.potential

WITH u, neighborIdx, direction
WHERE neighborIdx <> -1
MERGE (v:Node {rawIdx:neighborIdx})
MERGE (u)-[r:NEIGHBORING {direction:direction}]->(v)
"""
result = conn.run_query(insert_nodes_query)
insert_nodes_end = time()

# Elapsed time: 1200 [s] (20 min)
print(f"Elapsed time: {insert_nodes_end - insert_nodes_start} [s]")

# Add Voronoi map to database
print("Uploading Voronoi map...")
voronoi_map_start = time()

voronoi_map_query = """CALL apoc.load.json("database.json")
YIELD value
UNWIND value.voronoiMap as cell
UNWIND cell.nodes as cellNode
MERGE (vc:VoronoiCell {root:cell.root})

WITH vc, cellNode
MATCH (n:Node)
WHERE cellNode.rawIdx = n.rawIdx

MERGE (vc)-[:CONTAINS]->(n)
MERGE (n)-[:MEMBER_OF]->(vc)
"""

result = conn.run_query(voronoi_map_query)
voronoi_map_end = time()

# Elapsed time: 150 [s] (~2.5 min)
print(f"Elapsed time: {voronoi_map_end - voronoi_map_start} [s]")

# Add Room graph to database
print("Uploading room graph...")
room_graph_start = time()

room_graph_query = """CALL apoc.load.json("database.json")
YIELD value
UNWIND value.roomGraph as room
MERGE (r: Room {index:room.index})

WITH r, room
UNWIND room.cellIndices as cellIdx
MATCH (n: Node)
WHERE n.rawIdx = cellIdx

MERGE (r)-[:CONTAINS]->(n)
MERGE (n)-[:IS_INSIDE]->(r)

WITH r, room
UNWIND room.neighbors as neighbor
MERGE (r2: Room {index:neighbor.index})

MERGE (r)-[:NEIGHBORING {distance:neighbor.distance}]->(r2)
"""
# Neighobouring kısımlarını kendim yapacağım

result = conn.run_query(room_graph_query)
room_graph_end = time()

# Elapsed time:
print(f"Elapsed time: {room_graph_end - room_graph_start} [s]")


conn.close()
