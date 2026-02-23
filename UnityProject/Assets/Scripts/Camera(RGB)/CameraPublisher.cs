using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

public class CameraPublisher : MonoBehaviour
{
    public Camera cam;
    public string imageTopicName = "/camera/image_raw";
    public string infoTopicName = "/camera/camera_info";
    public int width = 640;
    public int height = 480;

    private ROSConnection ros;
    private Texture2D tex2D;
    private RenderTexture renderTexture;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(imageTopicName);
        ros.RegisterPublisher<CameraInfoMsg>(infoTopicName);

        renderTexture = cam.targetTexture;
        tex2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        // Flip the renderTexture vertically using Graphics.Blit
        RenderTexture flippedRT = RenderTexture.GetTemporary(width, height, 0, RenderTextureFormat.ARGB32);
        Graphics.Blit(renderTexture, flippedRT, new Vector2(1, -1), new Vector2(0, 1));

        RenderTexture.active = flippedRT;
        cam.Render();  // Ensure camera renders
        tex2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        tex2D.Apply();
        RenderTexture.active = null;
        RenderTexture.ReleaseTemporary(flippedRT);

        byte[] imageBytes = tex2D.GetRawTextureData();

        float timeNow = Time.time;
        int seconds = Mathf.FloorToInt(timeNow);
        int sec = seconds;
        float fractional = timeNow - seconds;
        uint nanosec = (uint)(fractional * 1e9f);

        // Publish Image
        ImageMsg imgMsg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg { sec = sec, nanosec = nanosec },
                frame_id = "camera_frame"
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(width * 3),
            data = imageBytes
        };
        ros.Publish(imageTopicName, imgMsg);

        // Publish CameraInfo
        CameraInfoMsg camInfoMsg = new CameraInfoMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg { sec = sec, nanosec = nanosec },
                frame_id = "camera_frame"
            },
            height = (uint)height,
            width = (uint)width,
            distortion_model = "plumb_bob",
            d = new double[] { 0, 0, 0, 0, 0 },
            k = new double[] { 1, 0, width / 2.0, 0, 1, height / 2.0, 0, 0, 1 },
            r = new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
            p = new double[] { 1, 0, width / 2.0, 0, 0, 1, height / 2.0, 0, 0, 0, 1, 0 }
        };
        ros.Publish(infoTopicName, camInfoMsg);
    }
}
