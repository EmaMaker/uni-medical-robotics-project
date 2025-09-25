using UnityEngine;

//[RequireComponent(typeof(CapsuleCollider))]
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
        if (string.IsNullOrEmpty(fingerId)) fingerId = FingerIdFromName(name);
    }

    void OnCollisionEnter(Collision collision)
    {
        // Per ricevere Collision:
        // - isTrigger=false
        // - almeno un Rigidbody NON-kinematic nel contatto (l’altro può essere kinematic)

        controller?.OnFingerTouchEnter(_self, collision.collider, fingerId);
    }

 
    void OnCollisionExit(Collision collision)
    {
        //controller?.OnFingerTouchExit(_self, collision.collider, fingerId);
    }

    static string FingerIdFromName(string n)
    {
        n = n.ToLower();
        if (n.Contains("thumb")) return "Thumb";
        if (n.Contains("index")) return "Index";
        if (n.Contains("middle")) return "Middle";
        if (n.Contains("ring")) return "Ring";
        if (n.Contains("pinky") || n.Contains("little")) return "Pinky";
        return "Palm";
    }

}


