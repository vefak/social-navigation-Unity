MATCH ()-[r:NEAR]->() DELETE r;
MATCH (h1:Human), (h2:Human)
WHERE h1.id < h2.id
WITH h1, h2,
     point({x:h1.position[0], y:h1.position[2]}) AS p1,
     point({x:h2.position[0], y:h2.position[2]}) AS p2
WHERE point.distance(p1, p2) < 2.0
CREATE (h1)-[:NEAR]->(h2);
CALL gds.graph.exists('humanGraph') YIELD exists
WITH exists
WHERE exists
CALL gds.graph.drop('humanGraph') YIELD graphName
RETURN graphName;
CALL gds.graph.project(
  'humanGraph',
  'Human',
  {
    NEAR: {
      orientation: 'UNDIRECTED'
    }
  }
);
CALL gds.wcc.write('humanGraph', {
  writeProperty: 'groupId'
});
CALL gds.graph.drop('humanGraph') YIELD graphName
RETURN graphName;
MATCH (g:Group)
DETACH DELETE g;
MATCH (h:Human)
WITH DISTINCT h.groupId AS groupId
CREATE (:Group {id: groupId});
MATCH (h:Human), (g:Group)
WHERE h.groupId = g.id
MERGE (h)-[:MEMBER_OF]->(g);
