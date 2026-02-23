using System.Collections.Generic;
using UnityEngine;
using System.IO;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;       // For geometry messages
using RosMessageTypes.Std;            // For standard ROS messages
using RosMessageTypes.BuiltinInterfaces; // For TimeMsg
using Unity.Robotics.Core;             // For TimeStamp
using RosMessageTypes.Unitycustommsg; // Import your custom message namespace

public class PoseTrailVisualizer_Traj : MonoBehaviour
{
    [SerializeField] Color m_Color = Color.green;
    [SerializeField] float m_Thickness = 0.1f;

    private List<Vector3> m_LoadedTrajectoryPoints = new List<Vector3>();
    private ROSConnection ros;
    private void Start()
    {
        //LoadTrajectoryFromFile();
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Point2DArrayMsg>("/reference_trajectory", OnTrajectoryReceived);

    }
    private void OnTrajectoryReceived(Point2DArrayMsg msg)
    {   
        
        // Generate a new random color
        m_Color = Random.ColorHSV();

        // Log the message type
        m_LoadedTrajectoryPoints.Clear();
        // Convert each Point2D in the message to a Unity Vector3
        foreach (var point in msg.points)
        {
            Vector3 unityPoint = new Vector3((float)point.x, (float)0.1, (float)point.y + (float)0.1); // height: 0.1
            m_LoadedTrajectoryPoints.Add(unityPoint);
            // Log each point's coordinates
        }
        DrawTrajectory(m_LoadedTrajectoryPoints);
    }

  

    private void DrawTrajectory(List<Vector3> points)
    {
        LineRenderer lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
        }

        lineRenderer.positionCount = points.Count;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default")); // Use a default material

        lineRenderer.startColor = m_Color;
        lineRenderer.endColor = m_Color;
        lineRenderer.startWidth = m_Thickness;
        lineRenderer.endWidth = m_Thickness;

        lineRenderer.SetPositions(points.ToArray());
    }
}
