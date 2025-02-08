using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OrbitingObstacle : MonoBehaviour
{
    [SerializeField]
    private float radius = 1.0F;

    [SerializeField]
    private float angularVelocity = 1.0F; // rad/s

    private Vector3 origin;

    // Start is called before the first frame update
    void Start()
    {
        origin = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        float angle = angularVelocity * Time.time;
        transform.position = origin + radius * new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
    }
}
