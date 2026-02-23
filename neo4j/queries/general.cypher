CREATE (Object:Object {name:"Object"})
CREATE (Dynamic:Dynamic {name:"Dynamic"})
CREATE (Human1:Human {type : 'Doctor', position : point({x : 0, y : 0}), orientation : 0})
CREATE (Human2:Human {type : 'Nurse', position : point({x : 1, y : 1}), orientation : 0})
CREATE
(Object) -[:hasSubclass]-> (Dynamic),
(Dynamic) -[:hasSubclass]->  (Human1),
(Dynamic) -[:hasSubclass]->  (Human2)
RETURN *



MATCH (r:Robot), (d:Dynamic)
CREATE (d) -[:hasSubclass]-> (r)
RETURN r, d

CREATE (Robot1:Robot) - [:hasAction] -> (Moving1:MovingStraight) 
RETURN * 

MATCH (N) 
RETURN N


CREATE (Object:Object {name:"Object"})

CREATE (Dynamic:Dynamic {name:"Dynamic"}),
(Human:Human {name:"Human"}),
(Group:Group {name:"Group"}),
(Robot:Robot {name:"Robot"})

CREATE (Static:Static {name:"Static"})
(Human:Human {name:"Human"}),

CREATE
(Object) -[:hasSubclass]-> (Dynamic),
(Dynamic) -[:hasSubclass]-> (Human),
(Dynamic) -[:hasSubclass]-> (Group),
(Dynamic) -[:hasSubclass]-> (Robot),
(Object) -[:hasSubclass]-> (Static)


RETURN *


MATCH (h1)
WHERE id(h1) = 93
SET h1.timestamp = 0.5
RETURN h1

