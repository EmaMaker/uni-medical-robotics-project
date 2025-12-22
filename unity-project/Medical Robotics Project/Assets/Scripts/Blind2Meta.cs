using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Security.Cryptography;
using UnityEngine;
using WeArt.Core;

public class Blind2Meta : MonoBehaviour
{

    public GameObject plane;
    public OVRHand hand;
    private PhysicsGraspController phGrasp = null;

    private WeArtHandGraspingSystem _graspingSystem;
    float timeArtifactStart = -1.0f;


    // Start is called before the first frame update
    void Start()
    {
        if (hand != null) phGrasp = hand.GetComponent<PhysicsGraspController>();
    }

    // https://docs.unity3d.com/ScriptReference/MonoBehaviour.OnRenderImage.html
    // Called once per render pass, when the camera has completed rendering
    void Update()
    {
        if (plane == null || hand == null || phGrasp == null) return;

        if (Input.GetKeyDown(KeyCode.C) || phGrasp._isGrabbing) {
            plane.SetActive(false);
            timeArtifactStart = -1.0f;
        }
        if (Input.GetKeyDown(KeyCode.R)) timeArtifactStart = Time.time + 1f;
        plane.SetActive(Time.time > timeArtifactStart && timeArtifactStart > 0);
    }
}
