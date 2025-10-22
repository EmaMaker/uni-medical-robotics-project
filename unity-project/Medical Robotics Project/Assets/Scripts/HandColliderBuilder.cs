using System.Collections.Generic;
using UnityEngine;
//using OculusSampleFramework; // se usi OVR
using static OVRSkeleton;

//Questo script prende dal prefab GameObject OVRSkeleton i joint delle dita e costruisce dei collider su ogni joint
//Crea all'interno della scena dei figli collider in tempo reale in base al tracking fatto dal visore
//va aggiunto al prefab OVRSkeleton

[RequireComponent(typeof(OVRSkeleton))]
public class HandColliderBuilder : MonoBehaviour
{
    [Header("Collider settings")]
    public bool buildOnStart = true;
    public float radius = 0.008f;           // 8 mm
    public float lengthScale = 0.9f;        // 90% della distanza osso->figlio
    public bool useTriggers = true;
    public LayerMask handLayer = default;

    [Header("Bones to include")]
    public List<OVRSkeleton.BoneId> included = new List<OVRSkeleton.BoneId> {
        OVRSkeleton.BoneId.Hand_Thumb1, OVRSkeleton.BoneId.Hand_Thumb2, OVRSkeleton.BoneId.Hand_Thumb3, OVRSkeleton.BoneId.Hand_ThumbTip,
        OVRSkeleton.BoneId.Hand_Index1, OVRSkeleton.BoneId.Hand_Index2, OVRSkeleton.BoneId.Hand_Index3, OVRSkeleton.BoneId.Hand_IndexTip,
        OVRSkeleton.BoneId.Hand_Middle1, OVRSkeleton.BoneId.Hand_Middle2, OVRSkeleton.BoneId.Hand_Middle3, OVRSkeleton.BoneId.Hand_MiddleTip,
        OVRSkeleton.BoneId.Hand_Ring1, OVRSkeleton.BoneId.Hand_Ring2, OVRSkeleton.BoneId.Hand_Ring3, OVRSkeleton.BoneId.Hand_RingTip,
        OVRSkeleton.BoneId.Hand_Pinky1, OVRSkeleton.BoneId.Hand_Pinky2, OVRSkeleton.BoneId.Hand_Pinky3, OVRSkeleton.BoneId.Hand_PinkyTip
    };

    private OVRSkeleton _skeleton;

    void Awake()
    {
        _skeleton = GetComponent<OVRSkeleton>();
    }

    void Start()
    {
        if (buildOnStart) Build();
    }

    [ContextMenu("Build Colliders")]
    public void Build()
    {
        if (_skeleton.Bones == null || _skeleton.Bones.Count == 0)
        {
            Debug.LogWarning("HandColliderBuilder: skeleton not ready yet.");
            return;
        }

        // pulizia eventuali colliders precedenti
        foreach (var col in GetComponentsInChildren<Collider>())
        {
            if (col.gameObject.name.StartsWith("HC_"))
                Destroy(col);
        }

        // mappa ossi
        var byId = new Dictionary<OVRSkeleton.BoneId, OVRBone>();
        foreach (var b in _skeleton.Bones) byId[b.Id] = b;

        foreach (var id in included)
        {
            if (!byId.TryGetValue(id, out var bone)) continue;

            Transform t = bone.Transform;
            Transform child = FindChildOf(t); // prova a prendere l’osso successivo per orientare la capsula

            // crea un GO figlio per non “sporcare” l’osso
            var go = new GameObject("HC_" + id.ToString());
            go.layer = LayerMaskToLayer(handLayer);
            go.transform.SetParent(t, false);

            CapsuleCollider cap = go.AddComponent<CapsuleCollider>();
            cap.isTrigger = useTriggers;
            cap.radius = radius;

            var rep = go.AddComponent<FingerContactController>();
            rep.controller = GetComponentInParent<PhysicsGraspController>();
            

            if (child != null)
            {
                Vector3 local = go.transform.InverseTransformPoint(child.position);
                float dist = local.magnitude * lengthScale;
                cap.height = Mathf.Max(dist, radius * 2f);
                // allinea l’asse Z della capsula lungo il vettore verso il figlio
                go.transform.rotation = Quaternion.LookRotation(child.position - t.position, t.up);
                cap.direction = 2; // 0=X,1=Y,2=Z
            }
            else
            {
                cap.height = radius * 2.0f;
                cap.direction = 2;
            }
        }
    }

    private Transform FindChildOf(Transform t)
    {
        
        return t.childCount > 0 ? t.GetChild(0) : null;
    }

    private int LayerMaskToLayer(LayerMask mask)
    {
        
        int value = mask.value;
        if (value == 0) return gameObject.layer;
        int layer = 0;
        while(value > 1) { value >>= 1; layer++; }
        return layer;
    }
}
