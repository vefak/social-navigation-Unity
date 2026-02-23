using UnityEngine;
using UnityEngine.AI;
using System.Collections;

[RequireComponent(typeof(NavMeshAgent))]
[RequireComponent(typeof(Animator))]
public class SmoothRandomWalker : MonoBehaviour
{
    public float walkRadius = 5f;
    public float decisionInterval = 3f;

    private NavMeshAgent agent;
    private Animator animator;
    private float timer;

    void Start()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();
        SetNewDestination();
        timer = 0f;
    }

    void Update()
    {
        // 1. Hızı animasyona geçir (blend tree için)
        float speed = agent.velocity.magnitude;
        animator.SetFloat("Speed", speed, 0.2f, Time.deltaTime); // damping

        // 2. Belirli aralıklarla yeni hedef seç
        timer += Time.deltaTime;
        if (timer >= decisionInterval || agent.remainingDistance < 0.5f)
        {
            SetNewDestination();
            timer = 0f;
        }

        // 3. Ajanın NavMesh’te olduğunu kontrol et
        if (!agent.isOnNavMesh)
        {
            Debug.LogWarning($"{gameObject.name} is off NavMesh!");
            return;
        }
    }

    void SetNewDestination()
    {
        Vector3 randomDirection = Random.insideUnitSphere * walkRadius;
        randomDirection += transform.position;

        if (NavMesh.SamplePosition(randomDirection, out NavMeshHit hit, walkRadius, NavMesh.AllAreas))
        {
            NavMeshPath path = new NavMeshPath();
            if (agent.CalculatePath(hit.position, path) && path.status == NavMeshPathStatus.PathComplete)
            {
                agent.SetDestination(hit.position);
            }
            else
            {
                // Hedef NavMesh'te ama yol geçersiz
                StartCoroutine(DelayRetryWithTurn());
            }
        }
        else
        {
            // NavMesh dışında bir hedef seçildi
            StartCoroutine(DelayRetryWithTurn());
        }
    }

    IEnumerator DelayRetryWithTurn()
    {
        // Opsiyonel: Dönme efekti
        float turnAngle = Random.Range(90f, 180f);
        transform.Rotate(0, turnAngle, 0);

        // Kısa bir bekleme sonra tekrar dene
        yield return new WaitForSeconds(0.5f);
        SetNewDestination();
    }
}
