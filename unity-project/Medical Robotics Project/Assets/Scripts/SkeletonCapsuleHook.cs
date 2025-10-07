
//This class allows me to have a reference of the OVRSkeleton that is created at runtime and this allows me to:
// - attach my script CollisionHandler, that handles actions to be taken on collision, to the capsule collider created
// - enable and disable this function if the capsule object was destroyed

using System.Linq;
using UnityEngine;

[DefaultExecutionOrder(+50)]
public class SkeletonCapsuleHook : MonoBehaviour
{
    public OVRSkeleton skeleton;

    bool hooked;

    void Awake()
    {
        if (!skeleton) skeleton = GetComponentInChildren<OVRSkeleton>(true);
    }

    void OnEnable() { TryHook(); }
    void Update() { if (!hooked) TryHook(); }
    void OnDisable() { hooked = false; }


    void TryHook()
    {
        if (!skeleton || !skeleton.IsInitialized) return;

        Transform capsulesRoot = skeleton.transform.Find("Capsules");

        // Fallback in case capsules is not a direct child
        if (!capsulesRoot)
        {
            foreach (var t in skeleton.GetComponentsInChildren<Transform>(true))
                if (t.name == "Capsules") { capsulesRoot = t; break; }
        }
        if (!capsulesRoot)
        {
            Debug.Log("Failed to get Capsules Root");
            return;
        }

        // Takes only the CapsuleCollider under the GameObject "Capsules"
        var caps = capsulesRoot.GetComponentsInChildren<CapsuleCollider>(true);
        if (caps == null || caps.Length == 0)
        {
            Debug.Log("No capsule collider exists");
            return;
        }

        var processed = new System.Collections.Generic.HashSet<GameObject>();

        // Add our CollisionHandler to all the rigid bodies which have a CapsuleCollider child.
        // Only add to distal capsule colliders (fingertip)
        foreach (var c in caps)
        {
            //if (!c) continue;
            // TODO: maybe drop the wrist and use the PalmAnchor as wrist collider?
            if (!c.name.Contains("Distal") && !c.name.Contains("Wrist")) continue;
            Debug.Log(c.name);

            Rigidbody rb = c.transform.parent.GetComponent<Rigidbody>();
            if (rb == null) continue;
            var rbGO = rb.gameObject;

            if (!processed.Contains(rbGO))
            {
                if (!rbGO.TryGetComponent<CollisionHandler>(out var hParent))
                    hParent = rbGO.AddComponent<CollisionHandler>();

                if (hParent && string.IsNullOrEmpty(hParent.fingerId))
                    hParent.fingerId = Utils.FingerDistalFromName(c.name);

                processed.Add(rbGO);
            }

        }

        hooked = true;
    }
}
