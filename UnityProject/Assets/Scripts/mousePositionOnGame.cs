using UnityEngine;
using System.Collections;

public class mousePositionOnGame : MonoBehaviour
{
    void Update()
    {
        if (Input.GetButtonDown("Fire1"))
        {
            Vector3 mousePos = Input.mousePosition;
            {

                Debug.Log("X:"+ mousePos.x);
                Debug.Log("Y:"+ mousePos.y);
            }
        }
    }
}