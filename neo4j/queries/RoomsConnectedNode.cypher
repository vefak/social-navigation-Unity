CREATE(r0:room {name:"MR Room"}) - [:isConnected] -> (r1:room {name:"Patience Room"})
CREATE(r1) - [:isConnected] -> (r0)
RETURN *