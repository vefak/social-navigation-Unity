using UnityEngine;

public class GetVelocityWithRigidbody : MonoBehaviour
{
    private Rigidbody rb; // Reference to the Rigidbody component

    void Start()
    {
        // Get the Rigidbody component attached to the character
        rb = GetComponent<Rigidbody>();
    }

    void Update()
    {
        // Access the velocity from the Rigidbody
        Vector3 velocity = rb.velocity;

        // Optional: Display the velocity in the console
        Debug.Log("RigidBody Velocity: " + velocity);
    }
}
