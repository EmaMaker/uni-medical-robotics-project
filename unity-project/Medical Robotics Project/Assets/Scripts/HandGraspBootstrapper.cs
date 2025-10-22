using System.Collections;
using System.Linq;
using UnityEngine;

public class HandGraspBootstrapper : MonoBehaviour
{
   /* [Header("Config")]
    public string grabbableTag = "object";   // il tag dei tuoi oggetti afferrabili
    public float grabThreshold = 0.70f;
    public float releaseThreshold = 0.55f;
    public float palmFacingDotMin = 0.15f;

    public bool useFixedJoint = true;
    public bool softConfigurable = false;

    IEnumerator Start()
    {
        // 1) aspetta che la mano venga instanziata (OVRHand + OVRSkeleton nei figli)
        OVRHand hand = null;
        OVRSkeleton skeleton = null;
        while (true)
        {
            hand = GetComponentInChildren<OVRHand>(true);
            skeleton = GetComponentInChildren<OVRSkeleton>(true);
            if (hand && skeleton) break;
            yield return null;
        }

        // 2) aspetta che lo scheletro sia popolato (bones disponibili)
        while (skeleton.Bones == null || skeleton.Bones.Count == 0)
            yield return null;

        // 3) trova un transform per il palmo (bone "Hand_Palm" se c'è, altrimenti il root della mano)
        Transform palmBone = skeleton.Bones
            .FirstOrDefault(b => b.Id.ToString().Contains("Palm"))?.Transform
            ?? skeleton.Bones.FirstOrDefault(b => b.Id == OVRSkeleton.BoneId.Hand_WristRoot)?.Transform
            ?? skeleton.transform;

        // 4) crea PalmAnchor come figlio del palmo
        var palmAnchorGO = new GameObject("PalmAnchor");
        palmAnchorGO.transform.SetParent(palmBone, false);
        // piccolo offset in avanti (fuori dal palmo) se vuoi
        // palmAnchorGO.transform.localPosition = new Vector3(0, 0, 0.02f);

        // 5) aggiungi (o recupera) HandColliderBuilder su OVRSkeleton
        var builder = skeleton.gameObject.GetComponent<HandColliderBuilder>();
        if (!builder) builder = skeleton.gameObject.AddComponent<HandColliderBuilder>();
        builder.useTriggers = true;
        builder.buildOnStart = true; // crea i colliders automaticamente

        // 6) aggiungi (o recupera) PhysicsGraspController sullo stesso GO dello skeleton
        var ctrl = skeleton.gameObject.GetComponent<PhysicsGraspController>();
        if (!ctrl) ctrl = skeleton.gameObject.AddComponent<PhysicsGraspController>();

        // 7) configura i riferimenti e i parametri principali
        ctrl.palmAnchor = palmAnchorGO.transform;
        ctrl.ovrHand = hand;
        ctrl.builder = builder;
        ctrl.grabbableTag = grabbableTag;

        ctrl.grabThreshold = grabThreshold;
        ctrl.releaseThreshold = releaseThreshold;
        ctrl.palmFacingDotMin = palmFacingDotMin;

        ctrl.useFixedJoint = useFixedJoint;
        ctrl.softConfigurable = softConfigurable;

        // (opzionale) altri parametri: throw multipliers, smoothing, ecc.
        ctrl.copyPalmVelocityOnRelease = true;
        ctrl.throwMultiplier = 1.0f;

        // 8) collega automaticamente i FingerContactReporter ai collider "HC_*"
        foreach (var col in skeleton.GetComponentsInChildren<Collider>(true))
        {
            if (!col.name.StartsWith("HC_")) continue;
            var rep = col.gameObject.GetComponent<FingerContactController>();
            if (!rep) rep = col.gameObject.AddComponent<FingerContactController>();
            rep.controller = ctrl;
        }

        Debug.Log("[HandGraspBootstrapper] Grasping pronto su " + gameObject.name);
    }*/
}
