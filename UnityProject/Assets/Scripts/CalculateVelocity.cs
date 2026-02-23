using UnityEngine;

public class CalculateVelocity : MonoBehaviour
{
    private Vector3 previousPosition; // To store the last frame's position
    private Vector3 velocity; // To store the calculated velocity

    void Start()
    {
        // Initialize the previous position to the starting position of the character
        previousPosition = transform.position;
    }

    void Update()
    {
        // Calculate the difference in position between frames
        Vector3 displacement = transform.position - previousPosition;

        // Calculate the velocity (displacement per time)
        velocity = displacement / Time.deltaTime;

        // Update the previous position for the next frame
        previousPosition = transform.position;

        // Optional: Display the velocity in the console
        Debug.Log("Velocity: " + velocity);
    }

    public Vector3 GetVelocity()
    {
        return velocity;
    }
}
