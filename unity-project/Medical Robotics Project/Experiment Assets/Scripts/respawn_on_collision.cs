using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class respawn_on_collision : MonoBehaviour
{
    public GameObject Falling_Object;
    private Vector3 Spawn_Position;
    

    void Start()
    { 
        Spawn_Position = Falling_Object.transform.position;
        Physics.gravity = new Vector3(0, -3f, 0);
    }


    //Upon collision with the ground, this GameObject will be reset respawn point
    void OnTriggerEnter(Collider other)
    {

        Falling_Object.transform.position = Spawn_Position;

        // Optional: Reset velocity if the object has a Rigidbody
        Rigidbody rb = GetComponent<Rigidbody>();
        rb.velocity = Vector3.zero;
    }


    // Update is called once per frame
    void Update()
    {
        
    }
}

    
