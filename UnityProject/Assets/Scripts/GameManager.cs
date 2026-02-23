using UnityEngine;

public class GameManager : MonoBehaviour
{
    public GameObject humanPrefab;
    public int numberOfHumans = 10;
    public Vector3 spawnAreaCenter = Vector3.zero;
    public Vector3 spawnAreaSize = new Vector3(20, 0, 20);

    [Header("ROS")]
    public string topicPrefix = "/human";   // will become /human1, /human2, ...

    void Start()
    {
        for (int i = 0; i < numberOfHumans; i++)
        {
            Vector3 randPos = spawnAreaCenter + new Vector3(
                Random.Range(-spawnAreaSize.x/2, spawnAreaSize.x/2),
                0,
                Random.Range(-spawnAreaSize.z/2, spawnAreaSize.z/2)
            );

            var human = Instantiate(humanPrefab, randPos, Quaternion.identity, transform);

            // Set unique topic on the spawned instance
            var pub = human.GetComponent<RosPublisherCustom>();
            if (pub == null) pub = human.GetComponentInChildren<RosPublisherCustom>();
            if (pub != null)
            {
                pub.customMsgTopic = $"{topicPrefix}{i+1}";
                // (optional) if your publisher uses the GameObject name as an ID
                human.name = $"Human_{i+1}";
            }
            else
            {
                Debug.LogWarning("RosPublisherCustom not found on spawned human prefab.");
            }

        }
    }
}
