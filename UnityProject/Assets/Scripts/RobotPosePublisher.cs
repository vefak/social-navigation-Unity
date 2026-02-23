using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "robot";

    // The game object
    public GameObject robot;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
    }
    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            
            // Create a Pose message
            PoseMsg robotPoseMsg = new PoseMsg();

            // Set the position 
            robotPoseMsg.position.x = robot.transform.position.x;
            robotPoseMsg.position.y = robot.transform.position.y;
            robotPoseMsg.position.z = robot.transform.position.z;
            // Set the orientation (quaternion)
            robotPoseMsg.orientation.x = robot.transform.rotation.x;
            robotPoseMsg.orientation.y = robot.transform.rotation.y;
            robotPoseMsg.orientation.z = robot.transform.rotation.z;
            robotPoseMsg.orientation.w = robot.transform.rotation.w;

            Debug.Log(robot.transform.position.x);
            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, robotPoseMsg);

            timeElapsed = 0;
        }
    }
}