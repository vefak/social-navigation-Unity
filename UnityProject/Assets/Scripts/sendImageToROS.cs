using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class sendImageToROS : MonoBehaviour
{
    public RenderTexture renderTexture;

    ROSConnection ros;
    public string topicName = "camera1";

    public int threshold = 1; // Binary threshold value
    public static ImageMsg testMsg = new ImageMsg();
    private Camera cam;

    // The game object
    // public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    // Start is called before the first frame update
    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        cam = GetComponent<Camera>();
        RenderTexture renderTexture = new RenderTexture(Screen.width, Screen.height, 24);
        cam.targetTexture = renderTexture;
        Texture2D texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGBA32, false);
        cam.Render();
        RenderTexture.active = renderTexture;
        texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);
        texture.Apply();
        cam.targetTexture = null;
        RenderTexture.active = null;
        Destroy(renderTexture);

        Texture2D flippedTexture = new Texture2D(texture.width, texture.height);
        for (int y = 0; y < texture.height; y++)
        {
            for (int x = 0; x < texture.width; x++)
            {
            flippedTexture.SetPixel(x, texture.height - y - 1, texture.GetPixel(x, y));
            }
        }
        flippedTexture.Apply();
            
   

            

        Color32[] pixels = flippedTexture.GetPixels32();
        byte[] bytes = new byte[pixels.Length];

        for (int i = pixels.Length - 1; i != 0 ; i--)
        {
            byte grayscaleValue = (byte)((pixels[i].r + pixels[i].g + pixels[i].b) / 3); // Convert to grayscale

            
            bytes[i] = grayscaleValue > threshold ? (byte)0 : (byte)255; // Apply binary threshold

        }


        // Finally send the message to server_endpoint.py running in ROS
        testMsg = new ImageMsg(new HeaderMsg(), (uint)renderTexture.height, 
                                            (uint)renderTexture.width, "mono8", 0x00, (uint)renderTexture.width, bytes);
    }

    // Update is called once per frame
    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            
            ros.Publish(topicName, testMsg);

            timeElapsed = 0;
        }
    }
    
}
