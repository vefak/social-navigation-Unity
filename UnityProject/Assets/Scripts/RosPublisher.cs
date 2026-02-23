using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;


public class RosPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string transform_topic = "";
    public string twist_topic = "";

    // The game object
    public GameObject game_object;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float time_elapsed;

    private Rigidbody rb; // Reference to the Rigidbody component

    void Start()
    {
        // start the ROS connection
        rb = GetComponent<Rigidbody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TransformStampedMsg>(transform_topic);
        ros.RegisterPublisher<TwistMsg>(twist_topic);

    }

    private void Update()
    {
        time_elapsed += Time.deltaTime;

        if (time_elapsed > publishMessageFrequency)
        {

            // Create a HeaderMsg with a simple placeholder for now (you could add a real timestamp)
            HeaderMsg header = new HeaderMsg();

            // The frame ID of the child object (this could be any string you want to identify this frame)
            string child_frame_id = game_object.name; // Use the GameObject's name or any string

            // Create the TransformMsg containing position and rotation
            TransformMsg transform_msg = new TransformMsg(
                new Vector3Msg(
                    game_object.transform.position.x,
                    game_object.transform.position.y,
                    game_object.transform.position.z
                ),
                new QuaternionMsg(
                    game_object.transform.rotation.x,
                    game_object.transform.rotation.y,
                    game_object.transform.rotation.z,
                    game_object.transform.rotation.w
                )         
            );

            // Create the TransformStampedMsg
            TransformStampedMsg game_object_msg = new TransformStampedMsg(
                header,             // HeaderMsg
                child_frame_id,     // Child frame ID (string)
                transform_msg       // TransformMsg
            );

             // Access the linear and angular velocity from the Rigidbody
            Vector3 linear_velocity = rb.velocity;
            Vector3 angular_velocity = rb.angularVelocity;

            // Create the TwistMsg with the full velocity vectors
            TwistMsg twist_msg = new TwistMsg(
                new Vector3Msg(linear_velocity.x, linear_velocity.y, linear_velocity.z),
                new Vector3Msg(angular_velocity.x, angular_velocity.y, angular_velocity.z)
            );
        


            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(transform_topic, game_object_msg);
            ros.Publish(twist_topic, twist_msg);
            time_elapsed = 0;
        }
    }
}