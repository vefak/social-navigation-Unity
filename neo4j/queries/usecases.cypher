CREATE (hospital:Environment {name: 'Hospital'})
CREATE (robot:Entity {name: 'Robot', type: 'autonomous'})
CREATE (patient:Entity {name: 'Patient', type: 'human'})
CREATE (staff:Entity {name: 'Hospital Staff', type: 'human'})
CREATE (visitor:Entity {name: 'Visitor', type: 'human'})
CREATE (room1:Location {name: 'Preparation Room'})
CREATE (room2:Location {name: 'Target Room'})
CREATE (corridor:Location {name: 'Corridor'})

// 2. User Story 1: Navigation through the hospital

MATCH (robot:Entity {name: 'Robot'}), (room1:Location {name: 'Preparation Room'}), (room2:Location {name: 'Target Room'})
MERGE (robot)-[:STARTS_FROM]->(room1)
MERGE (robot)-[:MOVES_TO]->(room2)
RETURN robot;

// 3. User Story 2: Robots trajectory intersects with a person

MATCH (robot:Entity {name: 'Robot'}), (human:Entity {type: 'human'})
MERGE (robot)-[:INTERSECTS_WITH {status: 'slows_down_or_stops'}]->(human)
RETURN robot, human;

//4. User Story 3: Robot's trajectory intersects with a group

MATCH (robot:Entity {name: 'Robot'}), (group:Group {name: 'Group of People'})
MERGE (robot)-[:INTERSECTS_WITH {status: 'stops'}]->(group)
RETURN robot, group;

// 5. User Story 4: Robot encounters blocking objects,

MATCH (robot:Entity {name: 'Robot'}), (object:Object {type: 'Obstacle'})
MERGE (robot)-[:ENCOUNTERS {distance: 'certain_distance'}]->(object)
RETURN robot, object;

// 6. User Story 5: Individual or group overtakes the robot

MATCH (robot:Entity {name: 'Robot'}), (person:Entity {type: 'human'})
MERGE (person)-[:OVERTAKES]->(robot)
RETURN robot, person;

// 7. User Story 6: Opposite direction encounters

MATCH (robot:Entity {name: 'Robot'}), (individual:Entity {type: 'human'})
MERGE (robot)-[:PASSES_OPPOSITE_DIRECTION]->(individual)
RETURN robot, individual;

// 8. User Story 7: Robot approaches a stationary group or individual

MATCH (robot:Entity {name: 'Robot'}), (stationary:Entity {type: 'human'})
MERGE (robot)-[:APPROACHES {distance: 'within_social_distance'}]->(stationary)
RETURN robot, stationary;

// 9. User Story 8: Robot follows a moving group or individual

MATCH (robot:Entity {name: 'Robot'}), (moving:Entity {type: 'human'})
MERGE (robot)-[:FOLLOWS {distance: 'safe_follow_distance'}]->(moving)
RETURN robot, moving;

//10. User Story 9: Human hits the robot

MATCH (robot:Entity {name: 'Robot'}), (person:Entity {type: 'human'})
MERGE (person)-[:COLLIDES_WITH {status: 'unintentional'}]->(robot)
RETURN robot, person;

// 11. User Story 10: Emergency situations

MATCH (robot:Entity {name: 'Robot'}), (safe_area:Location {type: 'Safe Area'})
MERGE (robot)-[:MOVES_TO {status: 'emergency'}]->(safe_area)
RETURN robot, safe_area;

// 12. User Story 11: Unauthorized areas for robot

MATCH (robot:Entity {name: 'Robot'}), (restricted_area:Location {type: 'Unauthorized'})
MERGE (robot)-[:AVOIDS]->(restricted_area)
RETURN robot, restricted_area;

// 13. User Story 12: Interaction with medical staff

MATCH (robot:Entity {name: 'Robot'}), (staff:Entity {type: 'human'})
MERGE (staff)-[:CHECKS_PATIENT_ON]->(robot)
RETURN robot, staff;

// 14. User Story 13: Specific hallways/corridors for patient transportation

MATCH (robot:Entity {name: 'Robot'}), (hallway:Location {name: 'Specific Hallway'})
MERGE (robot)-[:USES]->(hallway)
RETURN robot, hallway;

// 15. User Story 14: Robot’s trajectory intersects with another robot

MATCH (robot1:Entity {name: 'Robot'}), (robot2:Entity {name: 'Another Robot'})
MERGE (robot1)-[:INTERSECTS_WITH {status: 'slows_down_or_stops'}]->(robot2)
RETURN robot1, robot2;

// 16. User Story 15: A robot overtakes another robot

MATCH (robot1:Entity {name: 'Robot'}), (robot2:Entity {name: 'Another Robot'})
MERGE (robot2)-[:OVERTAKES]->(robot1)
RETURN robot1, robot2;

// 17. User Story 16: Opposite directions - robots passing each other

MATCH (robot1:Entity {name: 'Robot'}), (robot2:Entity {name: 'Another Robot'})
MERGE (robot1)-[:PASSES_OPPOSITE_DIRECTION]->(robot2)
RETURN robot1, robot2;

// 18. User Story 17: Robot follows another robot

MATCH (robot1:Entity {name: 'Robot'}), (robot2:Entity {name: 'Another Robot'})
MERGE (robot1)-[:FOLLOWS {distance: 'safe_follow_distance'}]->(robot2)
RETURN robot1, robot2;
