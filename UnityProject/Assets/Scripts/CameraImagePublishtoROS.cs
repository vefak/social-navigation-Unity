using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class CameraImagePublishtoROS : MonoBehaviour
{
    // Get Render Texture
    public RenderTexture renderTexture;
    // Texture object
    private Texture2D texture;    
    // ROS object
    ROSConnection ros;
    public string topicName = "unitycamera/occupancygridmap"; // Change topic name according to yours
    // Publishing very N seconds
    public float publishMessageFrequency = 0.1f;
    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;
    // Camera Object
    private Camera cam;
    // Start is called before the first frame update
    void Start()
    {
    // start the ROS connection
    ros = ROSConnection.GetOrCreateInstance();
    ros.RegisterPublisher<ImageMsg>(topicName);
    // Get the Camera component attached to the same GameObject as this script
    cam = GetComponent<Camera>();
    
    if (cam == null){
        Debug.LogError("Camera component not found. Make sure this script is attached to a GameObject with a Camera component.");
    }
    // Create renderTexture and texture objects to assign Camera image
    // Done here because we need only one time
    // renderTexture = new RenderTexture(Screen.width, Screen.height, 24);
    texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false); // Unity RGB24 -> OpenCV rgb8
    }

    // Destroy the RenderTexture when the script is destroyed or no longer needed
    
 

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {   
            // Rendering camera
            // Assign camera frame to texture 
            cam.targetTexture = renderTexture;
            cam.Render();
            RenderTexture.active = renderTexture;
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);
            texture.Apply();
            cam.targetTexture = null;
            RenderTexture.active = null;
            //Destroy(renderTexture);

            // Convert texture ROS msg
            ImageMsg testMsg = texture.ToImageMsg(new HeaderMsg());
            testMsg.encoding="rgb8"; 
            testMsg.width = (uint)texture.width;
            testMsg.height = (uint)texture.height;
            testMsg.is_bigendian=0;
            testMsg.step = (uint)(texture.width * 3);

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, testMsg);

            timeElapsed = 0;
        }
    }
    
}
