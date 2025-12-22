using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class respawn_on_collision : MonoBehaviour
{
    private Vector3 Spawn_Position;
    

    void Start()
    { 
        Spawn_Position = transform.position;
        Physics.gravity = new Vector3(0, -3f, 0);
    }


    //Upon collision with the ground, this GameObject will be reset respawn point
    void OnTriggerEnter(Collider other)
    {
        // Optional: Reset velocity if the object has a Rigidbody
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            rb.position = Spawn_Position;
            rb.velocity = Vector3.zero;

        }
        else transform.position = Spawn_Position;
    }
}

    
