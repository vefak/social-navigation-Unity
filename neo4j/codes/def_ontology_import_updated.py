import json
from neo4j import GraphDatabase, basic_auth

with open('/home/akmanadm/dev/neo4j/ilma/neo4j.json') as config_file:
    config = json.load(config_file)
    host = config['host']
    port = config['port']
    user = config['user']
    password = config['password']

# Connection with Neo4j
driver = GraphDatabase.driver('bolt://' + host + ':' + port, auth = basic_auth(user = user, password = password), encrypted = False)
session = driver.session()

class Neo4jOntologyDB:
    @staticmethod
    def clear_database():
        """
        Clear the database before reloading the ontology to delete the graph elements and removing existing constraints.
        """
        #NOTE: with closes the session
        #with session: 
        # If reload=True --> first delete database elements:
        session.run(""" MATCH (n) DETACH DELETE n; """)
        session.run(""" DROP CONSTRAINT n10s_unique_uri; """)

    @staticmethod
    def add_neo4j_labels(label):
        """
        Classify Neo4j nodes according to the core-elements by including neo4j-labels.
        """
        # Add the general label 'Class' for every element of the taxonomy:
        session.run(""" MATCH (n:n4sch__""" + label + """) SET n:Class """)

        # Include Neo4j-labels to classify the Classes:
        #NOTE: The query finds all the nodes with label specified by term string, and then finds all the nodes that are connected to those nodes at any level of connection depth
        #TODO: If there isn't a loop connection with the resulting nodes of the second query line then the path to that nodes is expended from a list to a sequence of rows (elemnts of the list)
        #NOTE: Finally to all the nodes that make up the path a term label (not as a node property) is added to the node
        for term in ['Object', 'Action', 'Context', 'Event']:
            session.run(""" MATCH (start:n4sch__""" + label + """:Class) WHERE start.n4sch__label='""" + term + """'
                            MATCH p=(start)<-[*]-(end:Class:n4sch__""" + label + """)
                            WHERE not (end)<--(:Class:n4sch""" + label + """)
                            UNWIND nodes(p) as pathNodes
                            SET pathNodes:""" + term + """ """)

    def add_metadata(version: str, title: str, language: str, creator: str, publisher: str, label: str,
                     description: str):
        """
        This functions adds metadata to the ontology
        """
        #NOTE: MERGE will either create or match a pattern; If a node already exists and is found the node parameters will be updated to the ones give in the query
        session.run(""" MERGE (n:Ontology:metadata)
                        SET n.version='""" + version + """', 
                            n.title='""" + title + """', 
                            n.language='""" + language + """',
                            n.creator='""" + creator + """',
                            n.publisher='""" + publisher + """',
                            n.label='""" + label + """', 
                            n.description='""" + description + """' """)

    @staticmethod
    def import_taxonomy(owl_file, Reload, ontology_label, serialization_format, version, title, language, creator, publisher, description):
        """
        Import the ontology into the Neo4j database using neosemantics (n10s) plugin.
        """
        if Reload is True:
            Neo4jOntologyDB.clear_database()
        else:
            pass
        session.run(""" CREATE CONSTRAINT n10s_unique_uri FOR (r:Resource) REQUIRE r.uri is UNIQUE; """)
        session.run(""" CALL n10s.graphconfig.init({classLabel: 'AGO', 
                                                    subClassOfRel: 'subClassOf', 
                                                    dataTypePropertyLabel: 'dataProperty', 
                                                    objectPropertyLabel: 'objectProperty', 
                                                    subPropertyOfRel: 'subPropertyOf'}); """)
        #TODO: Comment this out later
        owl_file = 'https://raw.githubusercontent.com/Vicomtech/video-content-description-VCD/master/ontologies/AGO/ontology_database/OWL/AGO1.1.0.owl'
        if 'http' in owl_file:
            session.run("""CALL n10s.onto.import.fetch('""" + owl_file + """', '""" + serialization_format + """');""")
        else:
            session.run("""CALL n10s.onto.import.fetch('file:///""" + owl_file + """', '""" + serialization_format + """');""")

        Neo4jOntologyDB.add_neo4j_labels(ontology_label)
        Neo4jOntologyDB.add_metadata(version, title, language, creator, publisher, ontology_label, description)
        session.close()
        print("Loaded ontology from: {owl_file}".format(owl_file = owl_file))