using System.Collections;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class CameraImageToOccupancyGrid : MonoBehaviour
{
    public RenderTexture renderTexture;
    private Texture2D texture;
    
    private ROSConnection ros;
    public string topicName = "occupancy_grid_map"; 
    public float publishMessageFrequency = 1.0f;
    private Camera cam;
    public float x = 0;
    public float y = 0;
    public float z = 0;
    public float xx = 0;
    public float yy = 0;
    public float zz = 0;
    public float ww = 1;


    public float resolution = 0.0375f; // 3.75cm per pixel 

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OccupancyGridMsg>(topicName);
        cam = GetComponent<Camera>();
        

        if (cam == null)
        {
            Debug.LogError("Camera component not found.");
        }

        texture = new Texture2D(renderTexture.width, renderTexture.height, TextureFormat.RGB24, false);

        // Start the coroutine for publishing occupancy grid
        StartCoroutine(PublishOccupancyGrid());
    }

    IEnumerator PublishOccupancyGrid()
    {
        while (true)
        {
            yield return new WaitForSeconds(publishMessageFrequency);

            // Capture image from the camera
            cam.targetTexture = renderTexture;
            cam.Render();
            RenderTexture.active = renderTexture;
            texture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0, false);
            texture.Apply(); // Upload to GPU
            cam.targetTexture = null;
            RenderTexture.active = null;

            // Convert and publish occupancy grid
            OccupancyGridMsg occupancyGridMsg = ConvertToOccupancyGrid(texture);
            ros.Publish(topicName, occupancyGridMsg);
        }
    }

    OccupancyGridMsg ConvertToOccupancyGrid(Texture2D texture)
    {
        int width = texture.width;
        int height = texture.height;
        sbyte[] gridData = new sbyte[width * height];
        Vector3 pos = cam.transform.position;
        Quaternion rot = cam.transform.rotation;
        // Get raw pixel data for fast processing
        Color32[] pixels = texture.GetPixels32(); 

        for (int i = 0; i < gridData.Length; i++)
        {
            float brightness = (pixels[i].r + pixels[i].g + pixels[i].b) / (3.0f * 255.0f); // Normalize to [0,1]

            gridData[i] = brightness < 0.65f ? (sbyte)100 : (sbyte)0; // 100 = occupied, 0 = free
        }

        return new OccupancyGridMsg
        { 
            header = new HeaderMsg { stamp = new TimeStamp(Clock.time), frame_id = "map" },
            info = new MapMetaDataMsg
            {
                resolution = resolution,
                width = (uint)width,
                height = (uint)height,
                origin = new PoseMsg
                {
                    position = new PointMsg { x = x, y = y, z = z },
                    orientation = new QuaternionMsg { x = xx, y = yy, z = zz, w = ww }
                }
            },
            data = gridData
        };
    }
}