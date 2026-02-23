using UnityEngine;

public class ImageToWorldConversion : MonoBehaviour
{
    // Reference to the camera
    public Camera targetCamera;

    // 2D image point (pixel coordinates)
    public Vector2 imagePoint = new Vector2(609f, 316f);

    // Start is called before the first frame update
    void Start()
    {
        if (targetCamera == null)
        {
            targetCamera = Camera.main; // Use the main camera if none is specified
        }

        if (targetCamera != null)
        {
            // Adjust the Y-coordinate to match Unity's coordinate system
            float adjustedY = Screen.height - imagePoint.y;

            // Convert 2D image point to world coordinates
            Vector3 worldPoint = targetCamera.ScreenToWorldPoint(new Vector3(imagePoint.x, adjustedY, 0f));

            // Display the results
            Debug.Log("Image Point: " + imagePoint);
            Debug.Log("World Point: " + worldPoint);
        }
        else
        {
            Debug.LogError("Camera not specified.");
        }
    }
}
