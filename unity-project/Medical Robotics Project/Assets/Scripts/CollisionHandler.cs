using UnityEngine;

public class CollisionHandler : MonoBehaviour
{
    public PhysicsGraspController controller; 


    [Tooltip("Finger ID, es. Thumb, Index, Middle, Ring, Pinky")]
    public string fingerId;

    Collider _self;

    void Awake()
    {
        _self = GetComponent<Collider>();
        if (!controller) controller = GetComponentInParent<PhysicsGraspController>();
        if (string.IsNullOrEmpty(fingerId)) fingerId = FingerDistalFromName(name);
    }

    void OnCollisionEnter(Collision collision)
    {
        controller?.OnFingerTouchEnter(_self, collision.collider, fingerId);
    }

 
    void OnCollisionExit(Collision collision)
    {
        controller?.OnFingerTouchExit(_self, collision.collider, fingerId);
    }

   static string FingerDistalFromName(string n)
    {
        n = n.ToLower();
        if (n.Contains("thumbdistal")) return "Thumb";
        if (n.Contains("indexdistal")) return "Index";
        if (n.Contains("middledistal")) return "Middle";
        if (n.Contains("ringdistal")) return "Ring";
        if (n.Contains("pinkydistal") || n.Contains("littledistal")) return "Pinky";
        if (n.Contains("wrist")) return "Palm";
        return "unknown";
    }

}


