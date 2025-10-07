using System.Linq;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;


//da aggiungere a OVRHandPrefab

[RequireComponent(typeof(OVRHand))]
public class PhysicsGraspController : MonoBehaviour
{
    [Header("Refs")]
    public Transform palmAnchor;
    public OVRHand ovrHand;
    public OVRHand handCopy = null;
    public OVRSkeleton ovrSkeleton;
    public OVRMesh savedMesh = null;
    public OVRSkeleton savedSkeleton = null;
    public OVRPlugin.HandState handStateAtGrasp;
    IList<OVRBone> Bones;
    private Vector3 PalmForward() => palmAnchor ? palmAnchor.transform.TransformDirection(Vector3.forward) : transform.TransformDirection(Vector3.forward);


    [Header("Release throw")]
    public bool copyPalmVelocityOnRelease = true;
    public float throwMultiplier = 1.0f;
    public float maxThrowSpeed = 12f;        
    public float maxThrowAngSpeed = 20f;     
    [Range(0f, 1f)] public float velSmoothing = 0.2f;
    [Range(0f, 1f)] public float angSmoothing = 0.2f;

    // Palm Velocity
    private Vector3 _lastPalmPos;
    private Quaternion _lastPalmRot;
    private Vector3 _palmVelocity;
    private Vector3 _palmAngularVelocity;


    // Fingers in contact with RB
    private readonly Dictionary<Rigidbody, HashSet<string>> _fingerContacts = new();
    private readonly Dictionary<Rigidbody, float> _fingerRelease = new();
    private readonly float RELEASE_COOLDOWN = 0.25f;
    //distance from rigidbody for each finger
    private readonly Dictionary<string, float> _fingerDistanceObj = new();

    //distance from rigidbody for each finger at beginning of grasping
    private readonly Dictionary<string, float> _fingerDistanceObjAtContact = new();
    
    //All Finger colliders
    private Dictionary<string, Collider> _fingerColliders = new();

    private bool _isGrabbing = false;
 
    private Rigidbody _currentBody;



    void Reset()
    {
        ovrHand = GetComponent<OVRHand>();
    }

    void Start()
    {
        if (!ovrHand) ovrHand = GetComponent<OVRHand>();
        ovrSkeleton = GetComponent<OVRSkeleton>();

        // Very ugly workaround: always disable ovrskeletonrenderer to avoid ghosts when creating a clone hand
        GetComponent<OVRSkeletonRenderer>().enabled = false;

        if (!palmAnchor)
        {
            // creating a new GameObject palmAnchor
            palmAnchor = new GameObject("PalmAnchor").transform;
            palmAnchor.SetParent(transform, false);
            palmAnchor.localPosition = new Vector3(0.0f, 0.0f, 0.08f);
            palmAnchor.localRotation = Quaternion.Euler(90.0f, 0.0f, 0.0f);
        }

        Transform capsulesRoot = ovrSkeleton.transform.Find("Capsules");

        if (!capsulesRoot)
        {
            foreach (var t in ovrSkeleton.GetComponentsInChildren<Transform>(true))
                if (t.name == "Capsules") { capsulesRoot = t; break; }
        }
        if (!capsulesRoot) return;

        // Takes only the CapsuleCollider under the GameObject "Capsules"
        var caps = capsulesRoot.GetComponentsInChildren<CapsuleCollider>(true);

        //Taking all the distal capsule colliders on the fingers
        foreach (var handler in caps)
        {
            string fingerId = FingerDistalFromName(handler.name);
            if (!_fingerColliders.ContainsKey(fingerId))
            {
                _fingerColliders[fingerId] = handler.GetComponent<Collider>();
            }
        }
    }


    private Collider GetFingerCollider(string fingerId)
    {
        return _fingerColliders.ContainsKey(fingerId) ? _fingerColliders[fingerId] : null;
    }


    void FixedUpdate()
    {
        if (!palmAnchor) return;

        var dt = Time.deltaTime;
        // Taking Linear and Angular Velocity and position and rotation from the palmAnchor
        // used to transfer palm velocity to object on release
        var rawVel = (palmAnchor.position - _lastPalmPos) / dt;
        _palmVelocity = Vector3.Lerp(rawVel, _palmVelocity, velSmoothing);

        Quaternion dq = palmAnchor.rotation * Quaternion.Inverse(_lastPalmRot);
        dq.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;
        var rawAngVel = axis * (angleDeg * Mathf.Deg2Rad / dt);
        _palmAngularVelocity = Vector3.Lerp(rawAngVel, _palmAngularVelocity, angSmoothing);

        _lastPalmPos = palmAnchor.position;
        _lastPalmRot = palmAnchor.rotation;

        List<Rigidbody> toRemove = new List<Rigidbody>();
        foreach (var finger in _fingerRelease)
        {
            if (Time.fixedTime - finger.Value > RELEASE_COOLDOWN)
            toRemove.Add(finger.Key);
        }
        foreach(var finger in toRemove) _fingerRelease.Remove(finger);

        if (_currentBody == null || !ovrSkeleton.IsDataHighConfidence) return;

        // TODO: only update this if data is high confidence (OVRHand)
        UpdateDst(_currentBody);

        //Release condition
        bool release = true;
        foreach (var p in _fingerDistanceObjAtContact)
        {
            //Check current finger distance from object w.r.t distance recorded at beginning of grasping
            //if (_fingerDistanceObj[p.Key] < p.Value - 0.005f)
            if (_fingerDistanceObj[p.Key] < p.Value + 0.0005f)
            {
                release = false;
                break;
            }
        }
        if (release)
        {
            EndGrab();
        }
    }

    //Updates the distance from each finger to the rigid body at any time
    void UpdateDst(Rigidbody rb)
    {
        if (rb == null) return;
        string[] fingerNames = { "Thumb", "Index", "Middle", "Ring", "Pinky" };

        //Getting each finger's distance from the touched rigidbody
        foreach (string s in fingerNames)
        {
            var fingerCol = GetFingerCollider(s);

            if (fingerCol != null)
            {
                Vector3 p1 = rb.ClosestPointOnBounds(fingerCol.transform.position);
                Vector3 p2 = fingerCol.bounds.ClosestPoint(rb.worldCenterOfMass);
                float dist = Vector3.Distance(p1, p2);
                _fingerDistanceObj[s] = dist;
            }
        }

    }

    // ---  Events from Finger Colliders (capsules) ---

    public void OnFingerTouchEnter(Collider finger, Collider other, string fingerId = null)
    {
        if (!other.attachedRigidbody) return;

        var rb = other.attachedRigidbody;

        if (!_fingerContacts.ContainsKey(rb)) _fingerContacts[rb] = new HashSet<string>();

        _fingerContacts[rb].Add(fingerId);
        TryBeginGrab(rb);
    }

    public void OnFingerTouchExit(Collider finger, Collider other, string fingerId = null)
    {
        if (!other.attachedRigidbody) return;

        var rb = other.attachedRigidbody;
        if (_fingerContacts.ContainsKey(rb))
        {
            _fingerContacts[rb].Remove(fingerId);
            if (_fingerContacts[rb].Count == 0) _fingerContacts.Remove(rb);
        }
    }

    // --- Logica di presa/rilascio ---

    void TryBeginGrab(Rigidbody rb)
    {
        if (_isGrabbing || !_fingerContacts.ContainsKey(rb) || _fingerRelease.ContainsKey(rb) || !palmAnchor || !ovrSkeleton.IsDataHighConfidence) return;

        HashSet<string> fingersInContact = _fingerContacts[rb];

        bool thumbInContact = fingersInContact.Contains("Thumb");
        bool palmInContact = fingersInContact.Contains("Palm");
        bool hasOtherFinger = false;
        foreach (var f in fingersInContact) { if ((f != "Thumb") && (f != "Palm")) { hasOtherFinger = true; break; } }


        // PALM FACING: the object has to be in front of the palm
        Vector3 toObj = (rb.worldCenterOfMass - palmAnchor.position).normalized;
        float facing = Vector3.Dot(PalmForward(), toObj);
        bool palmFacingOk = facing > 0.5;

        //Distance from the palm to the current object
        float dst = rb.ClosestPointOnBounds(palmAnchor.position).magnitude;


        // GRASPING CONDITION
        Debug.Log("---------" + facing);
        Debug.DrawLine(palmAnchor.position, toObj);
        Debug.DrawLine(palmAnchor.position, palmAnchor.position+PalmForward());
        bool ok = (((thumbInContact || (palmInContact ))) && hasOtherFinger && dst <= 1.1f && palmFacingOk);

        if (ok)
        {
            UpdateDst(rb);
            foreach (string s in fingersInContact)
            {
                if (s == "Palm" || s == "unknown") continue;
                _fingerDistanceObjAtContact[s] = _fingerDistanceObj[s];
            }

            // LEAVING IT HERE, check later
            /*string debugs = $"Grasping starting, conditions satisfied:\nThumb in contact: {thumbInContact}\nPalm In Contact: {palmInContact}\n";
            foreach (var f in _fingerDistanceObjAtContact)
            {
                Debug.Log($"Distance {f.Key} at start attempt: {f.Value}");
                debugs+=$"{f.Key} Distance: {f.Value}";
            }*/

            /*foreach (var f in _fingerDistanceObjAtContact)
            {
                if (f.Value > 0.03f)
                {
                    //Debug.Log($"{f.Key} too high: {f.Value}");
                    return;
                }
            }*/
            //Debug.Log(debugs);
            
            // Create a copy of the hand, attached to the hand itself as a child (to automatically update the pose)
            // and no OVRSkeleton to keep a ghost hand in the pose used to grasp the object
            if (handCopy == null)
            {
                // Disable rendering components on original hand
                enableRenderComponents(ovrHand, false);
                // Cannot destroy previous ovrskeletonrendererdata, as it leads to nullreferenceseven when ovrskeletonrenderer is disable
                // Ugly solution: never use ovrskeletonrenderer
                //Transform sr = ovrHand.transform.Find("SkeletonRenderer");
                //if (sr != null) Destroy(sr.gameObject);
                // Move all children of original hand to root of hierarcy to avoid creating duplicates in copy
                /*foreach (Transform child in ovrHand.gameObject.transform)
                {
                    child.SetParent(null, false);
                    list.Add(child);
                }*/
                // Create copy hand
                handCopy = Instantiate(ovrHand.gameObject, transform).GetComponent<OVRHand>();
                // Destroy PalmAnchor child of copy, so no duplicate objects are created
                Transform pa = handCopy.transform.Find("palmAnchor");
                if (pa != null) Destroy(pa.gameObject);
                // Disable rendering on copy (enable it when data is obtained)
                enableRenderComponents(handCopy, false);
                handCopy.GetComponent<PhysicsGraspController>().enabled = false;
                handCopy.GetComponent<SkeletonCapsuleHook>().enabled = false;
                // Disable capsule physics (property made public) and destroy existing capsules
                handCopy.GetComponent<OVRSkeleton>()._enablePhysicsCapsules = false;
                Transform ca = handCopy.transform.Find("Capsules");
                if (ca != null) Destroy(ca.gameObject);
                handCopy.medicalFirstUpdate = true;
            }
            BeginGrab(rb);
        }

    }

    List<Transform> list = new List<Transform>();
    private void Update()
    {
        // Let the copy do a first update the find the position of the hand, then disable skeleton
        if (handCopy != null && !(handCopy.medicalFirstUpdate))
        {
            enableRenderComponents(handCopy, true);
            handCopy.GetComponent<OVRSkeleton>().enabled = false;

            // After a first update is done, finally reattach all children to the original hand
            foreach (Transform child in list) child.SetParent(ovrHand.transform, false);
            // Then disable rendering for the original hand
            enableRenderComponents(ovrHand, false);
        }
    }


    private void BeginGrab(Rigidbody rb)
    {
        if (!rb) return;

        _currentBody = rb;

        // Zeroing the velocity on the attach 
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        rb.isKinematic = true;
        rb.useGravity = false;

        // TODO: disable colliders on the grasped object
        //rb.gameObject.GetComponent<Collider> ().enabled = false;
        rb.transform.SetParent(palmAnchor);

        _isGrabbing = true;
        ovrHand.medicalIsGrabbing = true;
        Debug.Log("!!!!!!!! GRASPING OBJECT !!!!!!!");
    }

    void EndGrab()
    {

        if (!_isGrabbing) return;

        //Transfering the palm velocity to the object on release (Throw feature) 
        if (_currentBody && copyPalmVelocityOnRelease)
        {
            var v = _palmVelocity * throwMultiplier;
            var w = _palmAngularVelocity;
            if (v.magnitude > maxThrowSpeed) v = v.normalized * maxThrowSpeed;
            if (w.magnitude > maxThrowAngSpeed) w = w.normalized * maxThrowAngSpeed;

            _currentBody.transform.SetParent(null);
            _currentBody.isKinematic = false;
            _currentBody.velocity = v;
            _currentBody.angularVelocity = w;
            _currentBody.useGravity = true;
            //_currentBody.gameObject.GetComponent<Collider> ().enabled = true;

            _fingerRelease[_currentBody] = Time.fixedTime;

        }
        _currentBody = null;
        _fingerContacts.Clear();
        _isGrabbing = false;
        ovrHand.medicalIsGrabbing = false;

        // Destroy copy hand and re-enable rendering
        enableRenderComponents(ovrHand, true);
        enableRenderComponents(handCopy, false);
        Destroy(handCopy.gameObject);
        handCopy = null;


        //handCopy = null;

        Debug.Log("!!!!!!!! NOT GRASPING !!!!!!!");
    }

    public void enableRenderComponents(OVRHand _ovrHand, bool _enable)
    {
        _ovrHand.GetComponent<OVRMeshRenderer>().enabled = _enable;
        //_ovrHand.GetComponent<OVRSkeletonRenderer>().enabled = _enable;
        _ovrHand.GetComponent<SkinnedMeshRenderer>().enabled = _enable;
        _ovrHand.GetComponent<OVRMesh>().enabled = _enable;
        /*if (!_enable)
        {
        } */
    }

    void OnDisable()
    {
        _currentBody = null;
        _isGrabbing = false;
    }

    public static string FingerDistalFromName(string n)
    {
        n = n.ToLower();
        if (n.Contains("thumbdistal")) return "Thumb";
        else if (n.Contains("indexdistal")) return "Index";
        else if (n.Contains("middledistal")) return "Middle";
        else if (n.Contains("ringdistal")) return "Ring";
        else if (n.Contains("pinkydistal") || n.Contains("littledistal")) return "Pinky";
        else if (n.Contains("wrist")) return "Palm";
        return "unknown";
    }
}