using UnityEngine;

public class CollisionHandler : MonoBehaviour
{
    public PhysicsGraspController controller; 


    [Tooltip("Finger ID, es. Thumb, Index, Middle, Ring, Pinky, Palm")]
    public string fingerId;

    Collider _self;

    void Awake()
    {
        _self = GetComponent<Collider>();
        if (!controller) controller = GetComponentInParent<PhysicsGraspController>();
        if (string.IsNullOrEmpty(fingerId)) fingerId = Utils.FingerDistalFromName(name);
    }

    void OnCollisionEnter(Collision collision)
    {
        controller?.OnFingerTouchEnter(_self, collision.collider, fingerId);
    }

 
    void OnCollisionExit(Collision collision)
    {
        controller?.OnFingerTouchExit(_self, collision.collider, fingerId);
    }


}


