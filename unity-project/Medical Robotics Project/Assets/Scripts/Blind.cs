using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEngine;

public class Blind : MonoBehaviour
{
    
    float timeArtifactStart = 0.0f;
    float timeArtifactDuration = 0.0f;
    bool artifactDataSet = false;
    readonly float timeArtifactMaxDuration = 2.5f;
    readonly float timeArtifactMinInterval = 2.0f;
    readonly float timeArtifactMaxInterval = 8.0f;
    public GameObject plane;

    // Start is called before the first frame update
    void Start()
    {
    }

    void ResetArtifact()
    {
        timeArtifactStart = Time.time + Random.Range(timeArtifactMinInterval, timeArtifactMaxInterval);
        timeArtifactDuration = Random.Range(0, timeArtifactMaxDuration);
        artifactDataSet = true;
        Debug.Log($"next lock-up will happen at t={timeArtifactStart} and last for {timeArtifactDuration}s");
    }

    // https://docs.unity3d.com/ScriptReference/MonoBehaviour.OnRenderImage.html
    // Called once per render pass, when the camera has completed rendering
    void Update()
    {
        if (plane == null) return;

        if(Time.time > timeArtifactStart && Time.time < timeArtifactStart+timeArtifactDuration){
            artifactDataSet = false;
            //Graphics.Blit(replacement, dest);
            plane.SetActive(true);

            Debug.Log("Locking up");

        }else{
            if (!artifactDataSet)
            {

                Debug.Log("Releasing");
                ResetArtifact();

            }

            plane.SetActive(false);
            //Graphics.Blit(source, dest);
            //Graphics.Blit(source, oldTexture);
        }

        // normal rendering
        // https://docs.unity3d.com/ScriptReference/Graphics.Blit.html
        // Graphics.Blit(source, dest);
    }
}
