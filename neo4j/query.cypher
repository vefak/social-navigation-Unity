// 1. Room connectivity
MATCH (r1:Room)-[conn:CONNECTED_TO]->(r2:Room)
RETURN r1 AS node1, r2 AS node2, conn AS rel
        
UNION
        
// 2. Robot -> HexCell
MATCH (robot:Robot)-[rel1:IS_AT]->(hex:HexCell)
WITH robot, hex, rel1
RETURN robot AS node1, hex AS node2, rel1 AS rel
        
UNION
        
// 3. Robot -> Room
MATCH (robot:Robot)-[rel2:IS_AT]->(room:Room)
RETURN robot AS node1, room AS node2, rel2 AS rel
        
UNION
        
// 4. Room -> HexCell (only for that hex from robot)
MATCH (robot:Robot)-[:IS_AT]->(hex:HexCell)<-[contains:CONTAINS_CELL]-(room:Room)
RETURN room AS node1, hex AS node2, contains AS rel


MATCH (r:Room)-[:CONTAINS_CELL]->(:HexCell)
RETURN count(DISTINCT r) AS numRoomsWithCells

MATCH (r:Room)-[:CONTAINS_CELL]->(c:HexCell)
RETURN r.index AS roomIndex, count(c) AS cellCount
ORDER BY roomIndex