using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Unitycustommsg;   // Your custom message type
using RosMessageTypes.Geometry;         // geometry_msgs
using RosMessageTypes.Std;              // std_msgs/Header
using RosMessageTypes.BuiltinInterfaces; // builtin_interfaces/Time
using Unity.Robotics.Core;              // TimeStamp
using System;
public class RosPublisherCustom : MonoBehaviour
{
    ROSConnection ros;
    public string customMsgTopic = "";

    public GameObject game_object;
    public float publishMessageFrequency = 0.1f;
    public string frameID = "map";

    private float time_elapsed;
    private Rigidbody rb;

    private Vector3 lastPosition;
    private Quaternion lastRotation;

    private const float THRESHOLD = 1e-3f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistTransformUnityMsg>(customMsgTopic);

        lastPosition = game_object.transform.position;
        lastRotation = game_object.transform.rotation;
    }

    void Update()
    {
        time_elapsed += Time.deltaTime;

        if (time_elapsed > publishMessageFrequency)
        {
            Vector3 currentPosition = game_object.transform.position;
            Quaternion currentRotation = game_object.transform.rotation;
            Debug.Log( $"{game_object.name} | " + 
            $"currentPosition = ({currentPosition.x:F2}, {currentPosition.y:F2}, {currentPosition.z:F2})" );
            Debug.Log( $"{game_object.name} | " + 
            $"currentRotation = ({currentRotation.x:F5}, {currentRotation.y:F5}, {currentRotation.z:F5})" );

     

            // Compute linear velocity
            Vector3 linear_velocity = (currentPosition - lastPosition) / time_elapsed;

            // Compute angular velocity using quaternion difference
            Quaternion deltaRotation = currentRotation * Quaternion.Inverse(lastRotation);
            deltaRotation.ToAngleAxis(out float angleInDegrees, out Vector3 rotationAxis);
            if (angleInDegrees > 180f) angleInDegrees -= 360f;

            Vector3 angular_velocity =
                rotationAxis.normalized * (angleInDegrees * Mathf.Deg2Rad) / time_elapsed;

            lastPosition = currentPosition;
            lastRotation = currentRotation;

            // Twist message
            TwistMsg twist_msg = new TwistMsg(
                new Vector3Msg(
                    RoundAndThreshold(linear_velocity.x),
                    RoundAndThreshold(linear_velocity.y),
                    RoundAndThreshold(linear_velocity.z)),
                new Vector3Msg(
                    RoundAndThreshold(angular_velocity.x),
                    RoundAndThreshold(angular_velocity.y),
                    RoundAndThreshold(angular_velocity.z))
            );


            Vector3 pos = game_object.transform.position;
            Vector3 eul = game_object.transform.eulerAngles;
            //Quaternion q = game_object.transform.rotation;


            Quaternion q = game_object.transform.rotation;
            float qx =  q.z;
            float qy = -q.x;
            float qz = -q.y;
            float qw =  q.w;

            TransformMsg transform_msg = new TransformMsg(
                    new Vector3Msg(
                        currentPosition.x,
                        currentPosition.y,
                        currentPosition.z
                    ),
                    new QuaternionMsg(
                        qx,
                        qy,
                        qz,
                        qw
                    )
                );



            Debug.Log( $"{game_object.name} | " + 
            $"Pos = ({pos.x:F2}, {pos.y:F2}, {pos.z:F2}) | " + 
            $"Euler = ({eul.x:F1}, {eul.y:F1}, {eul.z:F1}) | " );


            // Create header with ROS-compatible timestamp
            TimeStamp timeStamp = new TimeStamp(Time.time);

            HeaderMsg header = new HeaderMsg(
                timeStamp,   // Correct builtin_interfaces/Time
                frameID
            );

            TransformStampedMsg transform_stamped_msg = new TransformStampedMsg(
                header,
                game_object.name,
                transform_msg
            );

            TwistTransformUnityMsg custom_msg = new TwistTransformUnityMsg(
                twist_msg,
                transform_stamped_msg
            );

            ros.Publish(customMsgTopic, custom_msg);

            time_elapsed = 0;
        }
    }

    private float RoundAndThreshold(float value)
    {
        if (Mathf.Abs(value) < THRESHOLD)
            return 0f;

        return Mathf.Round(value * 10000f) / 10000f;
    }
}
