using UnityEngine;
using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Visualization;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;


public class BoundingBoxPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/bounding_boxes";
    public string frameID = "map";
    private GameObject targetObject;
    //private BoxCollider boxCollider;
    private CapsuleCollider capsuleCollider;
    public float x = 0;
    public float y = 0;
    public float z = 0;
    public float xx = 0;
    public float yy = 0;
    public float zz = 0;
    public float ww = 1;

    void Awake()
    {
        // Assign the object this script is attached to as the target
        targetObject = gameObject;
        // boxCollider = targetObject.GetComponent<CapsuleCollider>();
        capsuleCollider = targetObject.GetComponent<CapsuleCollider>();
        if (capsuleCollider == null)
        {
            Debug.LogError("BoundingBoxPublisher: Target object has no capsuleCollider.");
            enabled = false; // Disable the script to prevent errors
            return;
        }
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<MarkerArrayMsg>(topicName);
    }

    void Update()
    {
        if (targetObject == null || capsuleCollider == null)
        {
            Debug.LogWarning("BoundingBoxPublisher: No valid target or capsuleCollider found.");
            return;
        };
        
        // Vector3 objectPosition = targetObject.transform.position;
        Vector3 objectPosition = targetObject.transform.position + capsuleCollider.center;
        
        // Compute approximate bounding box from capsule
        Vector3 scale;
        switch (capsuleCollider.direction)
        {
            case 0: // X-axis
                scale = new Vector3(capsuleCollider.height, 2 * capsuleCollider.radius, 2 * capsuleCollider.radius);
                break;
            case 1: // Y-axis (default)
                scale = new Vector3(2 * capsuleCollider.radius, capsuleCollider.height, 2 * capsuleCollider.radius);
                break;
            case 2: // Z-axis
                scale = new Vector3(2 * capsuleCollider.radius, 2 * capsuleCollider.radius, capsuleCollider.height);
                break;
            default:
                scale = Vector3.one;
                break;
        }

        MarkerMsg bboxMarker = new MarkerMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg((int)Math.Floor(Time.time), (uint)((Time.time - Math.Floor(Time.time)) * 1e9)),
                frame_id = frameID
            },
            ns = "bounding_boxes",
            id = targetObject.GetInstanceID(),
            type = MarkerMsg.CUBE,
            action = MarkerMsg.ADD,
            pose = new PoseMsg
            {
                position = new PointMsg(objectPosition.x+x, 
                                        objectPosition.z+y,
                                        1+z),
                orientation = new QuaternionMsg(xx, 
                                            yy, 
                                            zz, 
                                            ww) 

            },
            scale = new Vector3Msg(1, 1, 2),
            color = new ColorRGBAMsg(0, 1, 1, 0.4f), // Blue and semi-transparent
            lifetime = new DurationMsg(0, 0),
            texture_resource = "",
            texture = new CompressedImageMsg()
            {
                header = new HeaderMsg(),
                format = "",
                data = new byte[0] // Empty image data
            },
            uv_coordinates = new UVCoordinateMsg[0], // Empty array
            text = "",
            mesh_resource = "",
            mesh_file = new MeshFileMsg()
            {
                filename = "",
                data = new byte[0] // Empty mesh data
            },
            mesh_use_embedded_materials = false
};

        MarkerArrayMsg markerArray = new MarkerArrayMsg { markers = new MarkerMsg[] { bboxMarker } };
        ros.Publish(topicName, markerArray);
    }
}
