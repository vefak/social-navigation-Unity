import json
import logging
import time
from abc import ABC, abstractmethod

from neo4j import GraphDatabase
from py2neo import Graph
from tqdm import tqdm


def batched(iterable, size):
    for i in range(0, len(iterable), size):
        yield iterable[i : i + size]


class GraphImporter(ABC):
    @abstractmethod
    def execute(self, query: str, parameters: dict = None): ...


class MemgraphImporter(GraphImporter):
    def __init__(self):
        from gqlalchemy import Memgraph

        self.db = Memgraph()

    def execute(self, query: str, parameters: dict = None):
        self.db.execute(query, parameters=parameters or {})


class Neo4jImporter(GraphImporter):
    def __init__(self, uri="bolt://localhost:7687", user="neo4j", password="12345678"):
        from neo4j import GraphDatabase

        self.driver = GraphDatabase.driver(uri, auth=(user, password))

        # Ensure py2neo graph object is initialized for visualization
        self.py2neo_graph = Graph(uri, auth=(user, password))

    def execute(self, query: str, parameters: dict = None):
        try:
            with self.driver.session() as session:
                session.run(query, parameters=parameters or {})
            return 1
        except Exception as e:
            print(f"❌ Cypher execution failed: {e}")
            return 0


def import_hexgrid(
    importer: GraphImporter, json_path, limit=None, batch_size=100, edge_batch_size=100
):
    with open(json_path, "r", encoding="utf-8") as f:
        hex_cells = json.load(f).get("hexGrid", [])

    if limit is not None:
        hex_cells = hex_cells[:limit]

    print(f"📦 Inserting {len(hex_cells)} HexCell nodes...")
    for batch in tqdm(
        list(batched(hex_cells, batch_size)), desc="HexCell node batches"
    ):
        props = [
            {
                "rawIdx": c["rawIdx"],
                "row": c["row"],
                "col": c["col"],
                "x": round(c["x"], 2),
                "y": round(c["y"], 2),
                "free": c["free"],
                "potential": round(c["potential"], 2),
            }
            for c in batch
        ]
        query = (
            "UNWIND $cells AS cell "
            "CREATE (:HexCell {rawIdx: cell.rawIdx, row: cell.row, col: cell.col, "
            "x: cell.x, y: cell.y, free: cell.free, potential: cell.potential})"
        )
        importer.execute(query, parameters={"cells": props})

    print("🔗 Creating edges...")
    raw_idx_set = {cell["rawIdx"] for cell in hex_cells}
    edges = [
        {"a": cell["rawIdx"], "b": b}
        for cell in hex_cells
        for b in cell.get("neighbors", [])
        if b != -1 and b in raw_idx_set
    ]

    for batch in tqdm(
        list(batched(edges, edge_batch_size)), desc="HexCell edge batches"
    ):
        query = (
            "UNWIND $pairs AS pair "
            "MATCH (a:HexCell {rawIdx: pair.a}), (b:HexCell {rawIdx: pair.b}) "
            "CREATE (a)-[:CONNECTED_TO]->(b)"
        )
        importer.execute(query, parameters={"pairs": batch})

    print("✅ HexGrid import complete.")


def import_rooms(
    importer: GraphImporter, json_path, batch_size=100, edge_batch_size=100
):
    with open(json_path, "r", encoding="utf-8") as f:
        room_graph = json.load(f).get("roomGraph", [])

    print(f"📦 Inserting {len(room_graph)} Room nodes...")
    for batch in tqdm(list(batched(room_graph, batch_size)), desc="Room node batches"):
        props = [{"index": r["index"]} for r in batch]
        query = "UNWIND $rooms AS room " "CREATE (:Room {index: room.index})"
        importer.execute(query, parameters={"rooms": props})

    edges = []
    for room in room_graph:
        for neighbor in room.get("neighbors") or []:
            edges.append(
                {
                    "a": room["index"],
                    "b": neighbor["index"],
                    "distance": neighbor["distance"],
                }
            )
        for cell_id in room.get("cellIndices") or []:
            edges.append({"a": room["index"], "cell": cell_id})

    neighbor_edges = [e for e in edges if "b" in e]
    cell_edges = [e for e in edges if "cell" in e]

    for batch in tqdm(
        list(batched(neighbor_edges, edge_batch_size)), desc="Room-Room edge batches"
    ):
        query = (
            "UNWIND $pairs AS pair "
            "MATCH (a:Room {index: pair.a}), (b:Room {index: pair.b}) "
            "CREATE (a)-[:CONNECTED_TO {distance: pair.distance}]->(b)"
        )
        importer.execute(query, parameters={"pairs": batch})

    for batch in tqdm(
        list(batched(cell_edges, edge_batch_size)), desc="Room-Cell edge batches"
    ):
        query = (
            "UNWIND $pairs AS pair "
            "MATCH (a:Room {index: pair.a}), (b:HexCell {rawIdx: pair.cell}) "
            "CREATE (a)-[:CONTAINS_CELL]->(b)"
        )
        importer.execute(query, parameters={"pairs": batch})

    print("✅ Room import complete.")


def import_voronoi(
    importer: GraphImporter, json_path, batch_size=100, edge_batch_size=100
):
    with open(json_path, "r", encoding="utf-8") as f:
        voronoi_map = json.load(f).get("voronoiMap", [])

    print(f"📦 Inserting {len(voronoi_map)} VoronoiRegion nodes...")
    for batch in tqdm(
        list(batched(voronoi_map, batch_size)), desc="Voronoi node batches"
    ):
        props = [{"root": r["root"]} for r in batch]
        query = (
            "UNWIND $regions AS region " "CREATE (:VoronoiRegion {root: region.root})"
        )
        importer.execute(query, parameters={"regions": props})

    edges = []
    for region in voronoi_map:
        for node in region.get("nodes") or []:
            edges.append(
                {
                    "root": region["root"],
                    "cell": node["rawIdx"],
                    "hexDist": node["hexDist"],
                    "centerDist": node["centerDist"],
                }
            )

    for batch in tqdm(
        list(batched(edges, edge_batch_size)), desc="Voronoi edge batches"
    ):
        query = (
            "UNWIND $pairs AS pair "
            "MATCH (a:VoronoiRegion {root: pair.root}), (b:HexCell {rawIdx: pair.cell}) "
            "CREATE (a)-[:HAS_NODE {hexDist: pair.hexDist, centerDist: pair.centerDist}]->(b)"
        )
        importer.execute(query, parameters={"pairs": batch})

    print("✅ Voronoi import complete.")


def main():
    # Choose your database
    use_neo4j = True

    if use_neo4j:
        importer = Neo4jImporter(
            uri="bolt://localhost:7687", user="neo4j", password="12345678"
        )

        indexes = [
            "CREATE INDEX hexcell_rawIdx_index IF NOT EXISTS FOR (n:HexCell) ON (n.rawIdx);",
            "CREATE INDEX room_index_index IF NOT EXISTS FOR (n:Room) ON (n.index);",
            "CREATE INDEX voronoi_root_index IF NOT EXISTS FOR (n:VoronoiRegion) ON (n.root);",
        ]

        # Drop old indexes
        drop_indexes = [
            "DROP INDEX hexcell_rawIdx_index IF EXISTS;",
            "DROP INDEX room_index_index IF EXISTS;",
            "DROP INDEX voronoi_root_index IF EXISTS;",
        ]

        # Optional: clear graph
        importer.execute("MATCH (n) DETACH DELETE n")

        for drop_query in drop_indexes:
            importer.execute(drop_query)

        for idx_query in indexes:
            importer.execute(idx_query)
    else:
        importer = MemgraphImporter()

    # Optional: clear graph
    # importer.execute("MATCH (n) DETACH DELETE n")

    start_time = time.time()

    import_hexgrid(importer, "database.json", batch_size=2000, edge_batch_size=10000)
    import_rooms(importer, "database.json", batch_size=200, edge_batch_size=10000)
    import_voronoi(importer, "database.json", batch_size=500, edge_batch_size=10000)

    # Delete isolated rooms
    delete_query = """
    MATCH (r:Room)
    WHERE size([x IN [(r)-[:CONNECTED_TO]-(other:Room) | other] | x]) < 3
    WITH collect(r) AS isolatedRooms
    UNWIND isolatedRooms AS r
    MATCH (r)-[:CONTAINS_CELL]->(h:HexCell)
    WITH isolatedRooms, collect(DISTINCT h) AS hexesToDelete
    UNWIND hexesToDelete AS h
    MATCH (v:VoronoiRegion)-[:HAS_NODE]->(h)
    WITH isolatedRooms, hexesToDelete, collect(DISTINCT v) AS voronoiToDelete
    FOREACH (r IN isolatedRooms | DETACH DELETE r)
    FOREACH (h IN hexesToDelete | DETACH DELETE h)
    FOREACH (v IN voronoiToDelete | DETACH DELETE v)
    """
    print("🧹 Deleting isolated rooms and related nodes...")
    importer.execute(delete_query)

    end_time = time.time()
    total_duration = end_time - start_time

    print(f"🕒 Total import time: {total_duration:.2f} seconds")

    # importer.close()


if __name__ == "__main__":
    main()
