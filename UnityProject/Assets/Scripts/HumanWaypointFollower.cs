using UnityEngine;
using UnityEngine.AI;
using System.Collections;

[RequireComponent(typeof(NavMeshAgent))]
[RequireComponent(typeof(Animator))]
public class HumanWaypointFollower : MonoBehaviour
{
    [Header("Waypoints")]
    public Transform[] waypoints;
    public float startDelay = 3f;
    public float stopDistance = 0.3f;

    [Header("Animation")]
    public string speedParam = "Speed"; // name of the float parameter in Animator
    public float animationSpeedMultiplier = 3.5f; // scale leg speed vs. nav speed
    public float minAnimSpeed = 0.8f;
    public float maxAnimSpeed = 3f;

    private int currentIndex = 0;
    private NavMeshAgent agent;
    private Animator animator;
    private bool isMoving = false;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        // ✅ Root Motion OFF for NavMeshAgent control
        animator.applyRootMotion = false;

        // Start walking after short delay
        StartCoroutine(StartAfterDelay());
    }

    IEnumerator StartAfterDelay()
    {
        yield return new WaitForSeconds(startDelay);
        if (waypoints.Length > 0)
        {
            isMoving = true;
            GoToNextWaypoint();
        }
    }

    void Update()
    {
        if (!isMoving || waypoints.Length == 0)
            return;

        float speed = agent.velocity.magnitude;

        // ✅ Feed NavMeshAgent speed into Animator
        animator.SetFloat(speedParam, speed * animationSpeedMultiplier);

        // ✅ Scale animation playback speed (longer strides)
        animator.speed = Mathf.Clamp(speed / 0.3f, minAnimSpeed, maxAnimSpeed);

        // ✅ Check waypoint reached
        if (!agent.pathPending && agent.remainingDistance < stopDistance)
        {
            currentIndex++;
            if (currentIndex >= waypoints.Length)
            {
                // Stop at last waypoint
                isMoving = false;
                agent.isStopped = true;
                animator.SetFloat(speedParam, 0f);
                animator.speed = 1f;
                return;
            }
            GoToNextWaypoint();
        }
    }

    void GoToNextWaypoint()
    {
        agent.isStopped = false;
        agent.SetDestination(waypoints[currentIndex].position);
    }
}
