using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

public class Radar_Simulated : MonoBehaviour
{
    public string topic = "/radar_scan"; // ROS topic name
    public int numRays = 90;             // How many beams to send
    public float fov = 90f;              // Field of view in degrees
    public float maxRange = 50f;         // How far each ray can go

    private ROSConnection ros;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topic);
    }

    void Update()
    {
        // Every frame, generate and publish a RADAR scan
        LaserScanMsg radarMsg = GenerateRadarScan();
        ros.Publish(topic, radarMsg);
    }

    LaserScanMsg GenerateRadarScan()
    {
        float[] ranges = new float[numRays];

        // Calculate angular spacing between rays
        float angleIncrement = fov / (numRays - 1) * Mathf.Deg2Rad;
        float angleMin = -fov / 2f * Mathf.Deg2Rad;

        Vector3 origin = transform.position;

        for (int i = 0; i < numRays; i++)
        {
            // Compute angle for this ray
            float angle = angleMin + i * angleIncrement;

            // Build direction vector from angle (2D: horizontal plane)
            Vector3 direction = transform.TransformDirection(new Vector3(Mathf.Cos(angle), 0, -Mathf.Sin(angle)));

            float distance;

            // Raycast to check for hits
            if (Physics.Raycast(origin, direction, out RaycastHit hit, maxRange))
            {
                distance = hit.distance;
                Debug.DrawRay(origin, direction * hit.distance, Color.green);
            }
            else
            {
                distance = maxRange;
                Debug.DrawRay(origin, direction * maxRange, Color.red);
            }

            ranges[i] = distance;
        }

        double now = Time.realtimeSinceStartupAsDouble;

        // Build the LaserScanMsg
        LaserScanMsg msg = new LaserScanMsg
        {
            header = new HeaderMsg
            {
                frame_id = "radar_frame",
                stamp = new TimeMsg
                {
                    sec = (int)now,
                    nanosec = (uint)((now - (int)now) * 1e9)
                }
            },
            angle_min = angleMin,
            angle_max = angleMin + numRays * angleIncrement,
            angle_increment = angleIncrement,
            range_min = 0.1f,
            range_max = maxRange,
            ranges = ranges
        };

        return msg;
    }
}
