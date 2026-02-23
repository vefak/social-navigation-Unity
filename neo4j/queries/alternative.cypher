# Action Classes Part

CREATE (Action:Action {name:"Action"})
CREATE (MovingManeuver:MovingManeuver {name:"MovingManeuver"})
CREATE (StationaryManeuver:StationaryManeuver {name:"StationaryManeuver"})

CREATE (Reaching:Reaching {name:"Reaching"})
CREATE (Intersecting:Intersecting {name:"Intersecting"})
CREATE (Overtaking:Overtaking {name:"Overtaking"})
CREATE (Passing:Passing {name:"Passing"})
CREATE (Joining:Joining {name:"Joining"})
CREATE (Approaching:Approaching {name:"Approaching"})
CREATE (Following:Following {name:"Following"})
CREATE (Moving:Moving {name:"Moving"})
CREATE (Colliding:Colliding {name:"Colliding"})
CREATE (Waiting:Waiting {name:"Waiting"})
CREATE (Checking:Checking {name:"Checking"})
CREATE (Standing:Standing {name:"Standing"})
CREATE (Sitting:Sitting {name:"Sitting"})
CREATE (Talking:Talking {name:"Talking"})
CREATE (Stopped:Stopped {name:"Stopped"})
CREATE (LyingDown:LyingDown {name:"LyingDown"})

CREATE (Action) -[:hasSubclass]-> (MovingManeuver),
(Action) -[:hasSubclass]-> (StationaryManeuver),
(MovingManeuver) -[:hasSubclass]-> (Reaching),
(MovingManeuver) -[:hasSubclass]-> (Intersecting),
(MovingManeuver) -[:hasSubclass]-> (Passing),
(MovingManeuver) -[:hasSubclass]-> (Overtaking),
(MovingManeuver) -[:hasSubclass]-> (Joining),
(MovingManeuver) -[:hasSubclass]-> (Approaching),
(MovingManeuver) -[:hasSubclass]-> (Following),
(MovingManeuver) -[:hasSubclass]-> (Moving),
(MovingManeuver) -[:hasSubclass]-> (Colliding),

(StationaryManeuver) -[:hasSubclass]-> (Waiting),
(StationaryManeuver) -[:hasSubclass]-> (LyingDown),
(StationaryManeuver) -[:hasSubclass]-> (Stopped),
(StationaryManeuver) -[:hasSubclass]-> (Sitting),
(StationaryManeuver) -[:hasSubclass]-> (Standing),
(StationaryManeuver) -[:hasSubclass]-> (Checking),
(StationaryManeuver) -[:hasSubclass]-> (Talking)

RETURN *


##############
## Object Classes

CREATE (Object:Object {name:"Object"})
CREATE (Dynamic:Dynamic {name:"Dynamic"}),
(Human:Human {name:"Human"}),
(Group:Group {name:"Group"}),
(Robot:Robot {name:"Robot"})

CREATE (Static:Static {name:"Static"}),
(Construction:Construction {name:"Construction"}),
(Furniture:Furniture {name:"Furniture"})

CREATE (Door:Door {name:"Door"}),
(Elevator:Elevator {name:"Elevator"}),
(Exit:Exit {name:"Exit"}),
(Hallway:Hallway {name:"Hallway"}),
(Room:Room {name:"Room"}),
(Wall:Wall {name:"Wall"}),
(Stairs:Stairs {name:"Stairs"}),
(Plant:Plant {name:"Plant"}),
(Armchair:Armchair {name:"Armchair"}),
(Bench:Bench {name:"Bench"}),
(Chair:Chair {name:"Chair"}),
(Counter:Counter {name:"Counter"}),
(Desk:Desk {name:"Desk"}),
(MR_Device:MR_Device {name:"MR_Device"}),
(Sofa:Sofa {name:"Sofa"}),
(Strecth:Strecth {name:"Strecth"})


CREATE
(Object) -[:hasSubclass]-> (Dynamic),
(Dynamic) -[:hasSubclass]-> (Human),
(Dynamic) -[:hasSubclass]-> (Group),
(Dynamic) -[:hasSubclass]-> (Robot),
(Object) -[:hasSubclass]-> (Static),
(Static) -[:hasSubclass]-> (Construction),
(Static) -[:hasSubclass]-> (Furniture),
(Construction) -[:hasSubclass]-> (Elevator),
(Construction) -[:hasSubclass]-> (Door),
(Construction) -[:hasSubclass]-> (Exit),
(Construction) -[:hasSubclass]-> (Hallway),
(Construction) -[:hasSubclass]-> (Room),
(Construction) -[:hasSubclass]-> (Wall),
(Construction) -[:hasSubclass]-> (Stairs),
(Furniture) -[:hasSubclass]-> (Plant),
(Furniture) -[:hasSubclass]-> (Armchair),
(Furniture) -[:hasSubclass]-> (Bench),
(Furniture) -[:hasSubclass]-> (Counter),
(Furniture) -[:hasSubclass]-> (Chair),
(Furniture) -[:hasSubclass]-> (Desk),
(Furniture) -[:hasSubclass]-> (MR_Device),
(Furniture) -[:hasSubclass]-> (Sofa),
(Furniture) -[:hasSubclass]-> (Strecth)


RETURN *
###################################3

MATCH(r1:RobotInstance {id:10})
MATCH(mr:RoomInstance {name: "Patience Room"})
CREATE(r1) - [:isOn] -> (mr)
RETURN * 

MATCH (hallway : Hallway)
CREATE (hallway0 : HallwayInstance {name: "Hallway"}) -[:isInstance] -> (hallway)
WITH hallway0
MATCH (room : RoomInstance)
CREATE (hallway0) -[:isConnected]-> (room)
CREATE (hallway0) <-[:isConnected]- (room)
RETURN *

MATCH (room : Room)
CREATE (room0: RoomInstance {name: "Patience Room"}) -[:isInstance] -> (room)
CREATE(room1: RoomInstance {name:"MR Room"}) - [:isInstance] -> (room)
RETURN *

MATCH (r : Robot)
CREATE (r1 : RobotInstance {name: "Bed Robot", position:point({x : 0, y:0}), orientation: 0, id:10}) -[:isInstance] -> (r)
RETURN *


