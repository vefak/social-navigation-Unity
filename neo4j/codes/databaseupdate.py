import json

import os
import numpy as np



from neo4j import GraphDatabase, basic_auth

# Read the config data to establish connection with the database
# NOTE: Make sure the database is running
with open('/home/akmanadm/dev/neo4j/ilma/neo4j.json') as config_file:
    config = json.load(config_file)
    host = config['host']
    port = config['port']
    user = config['user']
    password = config['password']
    
# Connection with Neo4j
driver = GraphDatabase.driver('bolt://' + host + ':' + port, auth = basic_auth(user = user, password = password), encrypted = False)
session = driver.session()

session.run(""" MATCH (n) DETACH DELETE n; """)


