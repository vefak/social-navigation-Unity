// Create Human nodes
CREATE (h1:Human {id: 'human1', name: 'John', position: [2, 3], orientation: 45}),
       (h2:Human {id: 'human2', name: 'Alice', position: [2.5, 3.1], orientation: 46}),
       (h3:Human {id: 'human3', name: 'Bob', position: [5, 8], orientation: 130}),
       (h4:Human {id: 'human4', name: 'Emma', position: [2.2, 3.4], orientation: 47}),
       (h5:Human {id: 'human5', name: 'Mustafa', position: [1.2,30.4], orientation: 280}),
       (h6:Human {id: 'human6', name: 'Murat', position: [11.2,110.4], orientation: 40}),
       (h7:Human {id: 'human7', name: 'Akman', position: [2, 3], orientation: 45})

// Carry forward the created nodes
WITH * 

// Match pairs of Human nodes
MATCH (h1:Human), (h2:Human)
WHERE id(h1) < id(h2)  // Avoid duplicate comparisons
  AND point.distance(point({x: h1.position[0], y: h1.position[1]}), point({x: h2.position[0], y: h2.position[1]})) <= 5.0  // Proximity check
  AND abs(h1.orientation - h2.orientation) <= 10  // Orientation similarity check

// Carry forward the matched pairs
WITH h1, h2 

// Create a Group node and link humans to it
MERGE (g:Group {id: 'group1'})  // Create a group node
MERGE (h1)-[:BELONGS_TO]->(g)   // Link human nodes to the group
MERGE (h2)-[:BELONGS_TO]->(g)

// Return the group and associated humans
RETURN g, h1, h2



//RETURN OUTPUTS

// Create group nodes based on proximity and orientation
MATCH (h1:Human), (h2:Human)
WHERE id(h1) < id(h2)  // Avoid duplicate comparisons
WITH h1, h2, 
     point.distance(point({x: h1.position[0], y: h1.position[1]}), point({x: h2.position[0], y: h2.position[1]})) AS distanceBetweenHumans,
     abs(h1.orientation - h2.orientation) AS orientationDifference
WHERE distanceBetweenHumans <= 1.0  // Proximity check
  AND orientationDifference <= 10  // Orientation similarity check
MERGE (g:Group {id: 'group1'})  // Create a group node
MERGE (h1)-[:BELONGS_TO]->(g)   // Link human nodes to the group
MERGE (h2)-[:BELONGS_TO]->(g)
RETURN h1.name AS Human1, h2.name AS Human2, distanceBetweenHumans, orientationDifference, g.id AS GroupID
