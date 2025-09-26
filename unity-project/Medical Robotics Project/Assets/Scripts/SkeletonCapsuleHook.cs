
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
        if (!skeleton)   skeleton   = GetComponentInChildren<OVRSkeleton>(true);
    }

    void OnEnable() { TryHook(); }
    void Update()   { if (!hooked) TryHook(); }
    void OnDisable(){ hooked = false; }


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
        if (!capsulesRoot) return;
        
        // Takes only the CapsuleCollider under the GameObject "Capsules"
        var caps = capsulesRoot.GetComponentsInChildren<CapsuleCollider>(true);
        if (caps == null || caps.Length == 0) return;

        var processed = new System.Collections.Generic.HashSet<GameObject>();

        foreach (var c in caps)
        {
            if (!c) continue;

            Transform t = c.transform.parent ?  c.transform.parent : c.transform;
            Rigidbody rb = null;

            while (t != null && t != capsulesRoot && !(rb = t.GetComponent<Rigidbody>()))
                t = t.parent;

            if (rb == null) continue;
            var rbGO = rb.gameObject;

            if (!processed.Contains(rbGO))
            {
                if (!rbGO.TryGetComponent<CollisionHandler>(out var hParent))
                    hParent = rbGO.AddComponent<CollisionHandler>();

                if (hParent && string.IsNullOrEmpty(hParent.fingerId))
                    hParent.fingerId = FingerDistalFromName(c.name);

                processed.Add(rbGO);
            }

        }

        hooked = true;
    }



    public static string FingerDistalFromName(string n)
    {
        n = n.ToLower();
        if (n.Contains("thumbdistal")) return "Thumb";
        if (n.Contains("indexdistal")) return "Index";
        if (n.Contains("middledistal")) return "Middle";
        if (n.Contains("ringdistal"))  return "Ring";
        if (n.Contains("pinkydistal") || n.Contains("littledistal")) return "Pinky";
        if (n.Contains("wrist")) return "Palm";
        return "unknown";
    }
}
