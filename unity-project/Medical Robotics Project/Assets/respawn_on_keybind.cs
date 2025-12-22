using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class respawn_on_keybind : MonoBehaviour
{
    private Vector3 Spawn_Position;
    

    void Start()
    { 
        Spawn_Position = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.O))
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
}
