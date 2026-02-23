using UnityEngine;

public class ImageCapture : MonoBehaviour
{
    public Camera captureCamera; // Reference to the camera you want to capture from

    void Update()
    {
        // Check for user input (e.g., press a key or button)
        if (Input.GetKeyDown(KeyCode.C))
        {
            CaptureImage();
        }
    }

    void CaptureImage()
    {
        // Create a RenderTexture to temporarily store the camera's rendering output
        RenderTexture renderTexture = new RenderTexture(Screen.width, Screen.height, 24);
        captureCamera.targetTexture = renderTexture;

        // Create a new Texture2D and read the pixels from the RenderTexture
        Texture2D screenshot = new Texture2D(1280, 720, TextureFormat.RGB24, false);
        captureCamera.Render();
        RenderTexture.active = renderTexture;
        screenshot.ReadPixels(new Rect(0, 0, 1280, 720), 0, 0);
        captureCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(renderTexture);

        // Convert the Texture2D to a byte array and save it as a PNG file
        byte[] bytes = screenshot.EncodeToPNG();
        System.IO.File.WriteAllBytes("Screenshot.png", bytes);

        Debug.Log("Screenshot captured!");
    }
}
