CREATE (s1:StartPoint {id: '10', name: 'Starting', position: [-27, -0.5], status:'stationary'}),
    (e1:EndPoint {id: '20', name: 'Ending', position: [-1, 10]}),
    (r1:RobotCurrentPos {id: 'robot1', name: 'Current', current_position: [-27, -0.5]})

CREATE (r1) -[:has]-> (s1),
    (r1) -[:has]-> (e1)

RETURN *