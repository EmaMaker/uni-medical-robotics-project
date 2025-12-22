using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Security.Cryptography;
using UnityEngine;
using WeArt.Core;

public class Blind2 : MonoBehaviour
{
    
    public GameObject plane;
    public GameObject hand;

    private WeArtHandGraspingSystem _graspingSystem;
    float timeArtifactStart = -1.0f;


    // Start is called before the first frame update
    void Start()
    {
        if(hand != null) _graspingSystem = hand.GetComponent<WeArtHandGraspingSystem>();
    }

    // https://docs.unity3d.com/ScriptReference/MonoBehaviour.OnRenderImage.html
    // Called once per render pass, when the camera has completed rendering
    void Update()
    {
        if (plane == null || _graspingSystem == null) return;

        if (Input.GetKeyDown(KeyCode.C) || _graspingSystem.GraspingState == GraspingState.Grabbed) {
            plane.SetActive(false);
            timeArtifactStart = -1.0f;
        }
        if (Input.GetKeyDown(KeyCode.R)) timeArtifactStart = Time.time + 1f;
        plane.SetActive(Time.time > timeArtifactStart && timeArtifactStart > 0);
    }
}
