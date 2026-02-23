// Create nodes for entities
CREATE (r:Robot {id: 'robot1', name: 'MR Robot'})
CREATE (p:Patient {id: 'patient1', name: 'John Doe'})
CREATE (s:Staff {id: 'staff1', name: 'Dr. Smith', profession: 'Doctor'})
CREATE (v:Visitor {id: 'visitor1', name: 'Jane Doe'})
CREATE (o:Obstacle {id: 'obstacle1', type: 'Static Object', description: 'Static'})
CREATE (room1:Location {id: 'room1', name: 'Preparation Room'})
CREATE (room2:Location {id: 'room2', name: 'MRI Room'})
CREATE (hallway:Location {id: 'hallway', name: 'Main Hallway'})

// Create path relationships for User Story 1
MATCH (r:Robot {id: 'robot1'}), (room1:Location {id: 'room1'}), (room2:Location {id: 'room2'}), (o:Obstacle {id: 'obstacle1'})
MERGE (r)-[:STARTS_FROM]->(room1)
CREATE (r)-[:AVOIDS {type: 'static'}]->(o)
CREATE (r)-[:MOVES_TO]->(room2)
RETURN r

// Create relationships for User Story 2
MATCH (r:Robot {id: 'robot1'}), (p:Patient {id: 'patient1'})
MERGE (r)-[:INTERSECTS]->(p)
MERGE (r)-[:DECIDES]->(:Decision {type: 'check_space', description: 'Check if enough space to pass'})
MERGE (r)-[:CONTINUES_PATH {action: 'pass_around', condition: 'enough space'}]->(:Path {id: 'path1', status: 'active'})
MERGE (r)-[:WAITS {reason: 'not enough space', action: 'signals to human or calls for help'}]->(:Assistance {id: 'assist1', status: 'awaiting'})
RETURN r

// Create relationships for User Story 3
MATCH (r:Robot {id: 'robot1'}), (g:Group {id: 'group1', type: 'People'})
MERGE (r)-[:INTERSECTS]->(g)
MERGE (r)-[:CHECKS_SPACE {type: 'group', description: 'Check if enough space around group'}]->(:PathCheck {id: 'check1', status: 'ongoing'})
MERGE (r)-[:REPLANS_PATH {status: 'if no space'}]->(:AlternativePath {id: 'alt_path1', status: 'searching'})
MERGE (r)-[:REQUESTS_HELP {type: 'signal', condition: 'no route available'}]->(:Assistance {id: 'assist2', status: 'required'})
RETURN r

// Create relationships for User Story 4
MATCH (r:Robot {id: 'robot1'}), (o:Obstacle {id: 'obstacle1'})
MERGE (r)-[:INTERSECTS]->(o)
MERGE (r)-[:CHECKS_SPACE {type: 'object', description: 'Check if enough space around object'}]->(:PathCheck {id: 'check2', status: 'ongoing'})
MERGE (r)-[:REPLANS_PATH {status: 'if no space'}]->(:AlternativePath {id: 'alt_path2', status: 'searching'})
MERGE (r)-[:REQUESTS_HELP {type: 'signal', condition: 'no route available'}]->(:Assistance {id: 'assist3', status: 'required'})
RETURN r

// Create relationships for User Story 5
MATCH (r:Robot {id: 'robot1'}), (g:Group {id: 'group1'})
MERGE (r)-[:OVERTAKEN_BY {actor: 'individual/group'}]->(g)
MERGE (r)-[:SLOWS_DOWN {reason: 'approaching from behind'}]->(:SpeedAdjustment {id: 'adjust1', speed: 'reduced'})
MERGE (r)-[:SIGNALS {type: 'warning', message: 'Movement signal to the people behind'}]->(:Communication {id: 'comm1', status: 'active'})
RETURN r

// Create relationships for User Story 10
MATCH (r:Robot {id: 'robot1'})
MERGE (r)-[:RECOGNIZES {type: 'emergency'}]->(:Situation {id: 'emergency1', status: 'active'})
MERGE (r)-[:MOVES_TO {type: 'safe_area'}]->(:Location {id: 'safe_area1', name: 'Nearest Safe Area'})
RETURN r

// Create relationships for User Story 11
MATCH (r:Robot {id: 'robot1'}), (u:Location {id: 'unauthorized1', name: 'Restricted Area'})
MERGE (r)-[:IDENTIFIES_AS_OBSTACLE {type: 'unauthorized_area'}]->(u)
MERGE (r)-[:AVOIDS {type: 'restricted'}]->(u)
RETURN r

