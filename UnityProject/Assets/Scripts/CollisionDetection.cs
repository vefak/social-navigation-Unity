using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class CollisionDetection : MonoBehaviour
{
    private ROSConnection ros;
    public string collisionTopic = "/collision_detected";
    public Collider myCollider;

    public static event System.Action OnCollisionDetected;
    public static event System.Action OnCollisionEnded;

    private bool collisionInProgress = false;

    void Start()
    {
        if (myCollider == null)
        {
            myCollider = GetComponent<Collider>();
            if (myCollider == null)
            {
                Debug.LogError("Collider not found on this GameObject!");
                return;
            }
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>(collisionTopic);

        // Ensure Rigidbody is attached to allow collision events
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
            rb.useGravity = false;  // Disable gravity if the robot should not fall
            rb.isKinematic = true;  // Make it kinematic if you don’t want physics forces applied
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (!collisionInProgress)
        {
            collisionInProgress = true;

            BoolMsg collisionMsg = new BoolMsg { data = true };
            ros.Publish(collisionTopic, collisionMsg);

            OnCollisionDetected?.Invoke();
            Debug.Log("Collision detected with: " + collision.gameObject.name);
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        collisionInProgress = true;
        BoolMsg collisionMsg = new BoolMsg { data = true };
        ros.Publish(collisionTopic, collisionMsg);
    }

    private void OnCollisionExit(Collision collision)
    {
        collisionInProgress = false;

        BoolMsg collisionMsg = new BoolMsg { data = false };
        ros.Publish(collisionTopic, collisionMsg);

        OnCollisionEnded?.Invoke();
        Debug.Log("Collision ended with: " + collision.gameObject.name);
    }
}
