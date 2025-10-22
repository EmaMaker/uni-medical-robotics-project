using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Random_Position : MonoBehaviour
{
    public GameObject Teleporting_Object;
    public GameObject Center;
    public GameObject Radius;

    void Start()
    {

    }

    void Update()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        Vector3 center = Center.transform.position;
        Vector3 radius_pos = Radius.transform.position;
        float radius = Vector3.Distance(center, radius_pos);

        if (Input.GetKeyDown(KeyCode.R))
        {
            Teleporting_Object.transform.position = center + radius * new Vector3(Random.Range(-0.5f, 0.5f), 0, Random.Range(-0.5f, 0.5f));

            rb.velocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;

            Debug.Log("R key was pressed.");
        }
    }
}
