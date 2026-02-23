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

# URI examples: "neo4j://localhost", "neo4j+s://xxx.databases.neo4j.io"
uri = "neo4j://localhost:7687"
user = "neo4j"
password = "flame-chemist-vienna-nova-student-5397"
conn = Neo4jConnection(uri, user, password)

# Enter nodes into database
print('Inserting hex cells into the database...')
insert_nodes_start = time()

insert_nodes_query = """CALL apoc.load.json("database.json")
YIELD value
UNWIND value.hexGrid as hex

MERGE (n:Node {rawIdx:hex.rawIdx})
SET n.col=hex.col, n.free=hex.free, n.row=hex.row, n.x=hex.x, n.y=hex.y
"""
result = conn.run_query(insert_nodes_query)
insert_nodes_end = time()

# Elapsed time: 100 [s] (~2 min)
print(f'Elapsed time: {insert_nodes_end - insert_nodes_start} [s]')

# Connect nodes to neighbors
print('Connecting neighboring cells...')
connecting_neighbors_start = time()

connecting_neighbors_query = """CALL apoc.load.json("database.json")
YIELD value
UNWIND value.hexGrid as hex
UNWIND range(0, size(hex.neighbors) - 1) as direction
WITH hex, hex.neighbors[direction] as neighborIdx, direction

MATCH (u:Node {rawIdx: hex.rawIdx})
MATCH (v:Node {rawIdx: neighborIdx})

MERGE (u)-[r:NEIGHBORING {direction:direction}]->(v)
"""

result = conn.run_query(connecting_neighbors_query)
connecting_neighbors_end = time()

# Elapsed time: 1500 [s] (25 min)
print(f'Elapsed time: {connecting_neighbors_end - connecting_neighbors_start} [s]')

# Add Voronoi map to database
print('Uploading Voronoi map...')
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
print(f'Elapsed time: {voronoi_map_end - voronoi_map_start} [s]')

conn.close()

