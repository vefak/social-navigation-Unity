using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Unitycustommsg; // Import your custom message namespace
using RosMessageTypes.Geometry;       // For geometry messages
using RosMessageTypes.Std;            // For standard ROS messages
using RosMessageTypes.BuiltinInterfaces; // For TimeMsg
using Unity.Robotics.Core;             // For TimeStamp

public class RosPublisherCustomas: MonoBehaviour
{
    ROSConnection ros;
    public string customMsgTopic ="";

    // The game object
    public GameObject game_object;
    public float publishMessageFrequency = 0.1f;
    public string frameID = "map";

    private float time_elapsed;
    private Rigidbody rb;
    
    private Vector3 lastPosition;
    private Quaternion lastRotation;
    
    // Threshold for zeroing out small values
    private const float THRESHOLD = 1e-3f;

    void Start()
    {
        // Start the ROS connection
        rb = GetComponent<Rigidbody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistTransformUnityMsg>(customMsgTopic); // Register custom message
        lastPosition = game_object.transform.position;
        lastRotation = game_object.transform.rotation;
    }

    void Update()
    {
        time_elapsed += Time.deltaTime;

        if (time_elapsed > publishMessageFrequency)
        {
            // Create the message
            //Vector3 linear_velocity = rb.velocity;
            //Vector3 angular_velocity = rb.angularVelocity;

            Vector3 currentPosition = game_object.transform.position;
            Quaternion currentRotation = game_object.transform.rotation;

            Vector3 linear_velocity = (currentPosition - lastPosition) / time_elapsed;
            
            Quaternion deltaRotation = currentRotation * Quaternion.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out float angleInDegrees, out Vector3 rotationAxis);
            if (angleInDegrees > 180f) angleInDegrees -= 360f; // Shortest rotation path
            Vector3 angular_velocity = rotationAxis.normalized * (angleInDegrees * Mathf.Deg2Rad) / time_elapsed;



            lastPosition = currentPosition;
            lastRotation = currentRotation;
            // Create the TwistMsg
            TwistMsg twist_msg = new TwistMsg(
                new Vector3Msg( RoundAndThreshold(linear_velocity.x), 
                                RoundAndThreshold(linear_velocity.y), 
                                RoundAndThreshold(linear_velocity.z)),
                new Vector3Msg( RoundAndThreshold(angular_velocity.x), 
                                RoundAndThreshold(angular_velocity.y), 
                                RoundAndThreshold(angular_velocity.z))
            );

            // Create the TransformStampedMsg
            TransformMsg transform_msg = new TransformMsg(
                new Vector3Msg(
                    game_object.transform.position.x,
                    game_object.transform.position.y,
                    game_object.transform.position.z
                ),
                new QuaternionMsg(
                    game_object.transform.rotation.eulerAngles.x * Mathf.Deg2Rad,
                    game_object.transform.rotation.eulerAngles.y * Mathf.Deg2Rad,
                    game_object.transform.rotation.eulerAngles.z * Mathf.Deg2Rad,
                    game_object.transform.rotation.w * Mathf.Deg2Rad
                )
            );
            
            // Get the simulation time from Unity and convert it to a ROS TimeMsg using TimeStamp
            TimeStamp timeStamp = new TimeStamp(Time.time);

            // Create the HeaderMsg with the timestamp
            HeaderMsg header = new HeaderMsg(
                timeStamp, // TimeStamp automatically converts to TimeMsg
                frameID // frame_id
            );

            TransformStampedMsg transform_stamped_msg = new TransformStampedMsg(
                header, game_object.name, transform_msg
            );

            // Create the CustomTwistTransformMsg
            TwistTransformUnityMsg custom_msg = new TwistTransformUnityMsg(
                twist_msg,
                transform_stamped_msg
            );

            // Publish the custom message
            ros.Publish(customMsgTopic, custom_msg);
            time_elapsed = 0;
        }
    }

// Helper function to round and threshold values
    private float RoundAndThreshold(float value)
    {
        // Zero out values smaller than the threshold
        if (Mathf.Abs(value) < THRESHOLD)
            return 0f;

        // Round to four decimal places
        return Mathf.Round(value * 10000f) / 10000f;
    }


}



