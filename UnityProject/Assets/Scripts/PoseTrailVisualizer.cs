using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class PoseTrailVisualizer : MonoBehaviour
{
    [SerializeField] Color m_Color = Color.green;
    [SerializeField] float m_Thickness = 0.1f;
    [SerializeField] string filePath = "TrajectoryData.txt"; 

    private List<Vector3> m_LoadedTrajectoryPoints = new List<Vector3>();

    private void Start()
    {
        LoadTrajectoryFromFile();
        DrawTrajectory(m_LoadedTrajectoryPoints);
    }

    private void LoadTrajectoryFromFile()
{
    m_LoadedTrajectoryPoints.Clear();

    // Ensure file exists
    if (!File.Exists(filePath))
    {
        Debug.LogError("Trajectory file not found at path: " + filePath);
        return;
    }

    // Read each line of the file
    string[] lines = File.ReadAllLines(filePath);
    foreach (string line in lines)
    {
        // Split the line into parts and check if there are at least two columns
        string[] parts = line.Split(' ');
        if (parts.Length >= 2 && float.TryParse(parts[0], out float x) && float.TryParse(parts[1], out float y))
        {
            Vector3 point = new Vector3(x, (float)0.1, y); // Assuming Z = 0
            m_LoadedTrajectoryPoints.Add(point);
        }
        else
        {
            Debug.LogWarning("Invalid line format or insufficient columns: " + line);
        }
    }
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
