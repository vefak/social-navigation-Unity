using System.Collections.Generic;
using UnityEngine;

public class PathFollower : MonoBehaviour
{
    [Header("Path Settings")]
    public List<Transform> pathPoints;

    [Header("Movement Settings")]
    public float moveSpeed = 2.0f;
    public float rotationSpeed = 5.0f; // Higher = faster rotation

    private int currentTargetIndex = 0;

    void Update()
    {
        if (pathPoints == null || pathPoints.Count == 0) return;

        Transform targetPoint = pathPoints[currentTargetIndex];
        Vector3 targetPos = targetPoint.position;

        // Move towards target
        transform.position = Vector3.MoveTowards(transform.position, targetPos, moveSpeed * Time.deltaTime);

        // Rotate smoothly toward target
        Vector3 direction = (targetPos - transform.position).normalized;
        if (direction != Vector3.zero)
        {
            Quaternion lookRotation = Quaternion.LookRotation(direction);
            transform.rotation = Quaternion.Slerp(transform.rotation, lookRotation, rotationSpeed * Time.deltaTime);
        }

        // Go to next point if close enough
        if (Vector3.Distance(transform.position, targetPos) < 0.1f)
        {
            currentTargetIndex++;
            if (currentTargetIndex >= pathPoints.Count)
            {
                // Loop or stop
                currentTargetIndex = 0; // or enabled = false;
            }
        }
    }
}
