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

    public GameObject cube;                 
    
    [Header("Filters")] 
    public string grabbableTag = "";    // metti nome nella stringa o se lasci vuoto -> ignora

    [Header("Thresholds (with hysteresis)")] //anti-flicker
    [Range(0, 1)] public float grabThreshold = 0.70f;
    [Range(0, 1)] public float releaseThreshold = 0.55f;
    public int debounceFrames = 2;         

    [Header("Palm facing")]
    [Range(-1f, 1f)] public float palmFacingDotMin = 0.15f; // se >0 l'oggetto è davanti al palmo

    public int minSustainFrames = 1; //min frame necessary to start grasping !!NOT USED NOW

    // Se il forward del palmAnchor punta 'fuori dal palmo', va bene così.
    // Altrimenti, cambia qui l'asse usato per il test.
    private Vector3 PalmForward() => palmAnchor ? palmAnchor.forward : transform.forward;


    //[Header("Joint Mode")]
    //public bool useFixedJoint = true;       // JOINT USED
    //public bool softConfigurable = false;   

    /*[Header("Joint Tuning (Configurable)")]
    public float positionSpring = 2000f;
    public float positionDamper = 80f;
    public float rotationSpring = 3000f;
    public float rotationDamper = 120f;*/
    public float breakForce = 1000f;
    public float breakTorque = 1000f;

    [Header("Release throw")]
    public bool copyPalmVelocityOnRelease = true;
    public float throwMultiplier = 1.0f;
    public float maxThrowSpeed = 12f;        // clamp velocità lineare
    public float maxThrowAngSpeed = 20f;     // clamp rad/s
    [Range(0f, 1f)] public float velSmoothing = 0.2f;
    [Range(0f, 1f)] public float angSmoothing = 0.2f;

    // Palm Velocity
    private Vector3 _lastPalmPos;
    private Quaternion _lastPalmRot;
    private Vector3 _palmVelocity;
    private Vector3 _palmAngularVelocity;
    

    // Fingers in contact with RB
    private readonly Dictionary<Rigidbody, HashSet<string>> _fingerContacts = new();

    private bool _isGrabbing = false;
    private int _framesStable = 0;
    private Rigidbody _currentBody;
    private Joint _joint; // FixedJoint o ConfigurableJoint

    void Reset()
    {
        ovrHand = GetComponent<OVRHand>();
    }

    void Start()
    {
        if (!ovrHand) ovrHand = GetComponent<OVRHand>();
        var skel = GetComponent<OVRSkeleton>();
        cube = GameObject.Find("Stick");

        if (!palmAnchor)
        {
            // creating a new GameObject palmAnchor
            palmAnchor = new GameObject("PalmAnchor").transform;
            palmAnchor.SetParent(transform, false);
            palmAnchor.localPosition = new Vector3(0.0f, 0.0f, 0.08f);
            // TODO: works for left hand. right hand has to be rotated the other way
            palmAnchor.localRotation = Quaternion.Euler(90.0f, 0.0f, 0.0f);
            //cube.transform.SetParent(palmAnchor, false);
            //cube.transform.localPosition = new Vector3(0.0f, 0.0f, 0.0f);
            //cube.transform.localRotation = Quaternion.identity;
            
        }

        StartCoroutine(AlignPalmAnchor(skel));
    }

    IEnumerator AlignPalmAnchor(OVRSkeleton skel)
    {
        // wait for skeleton initialization
        while (!skel || !skel.IsInitialized) yield return null;

        // finding the Hand_Bone_Palm -> PalmAncor alligned with Palm Bone
        //var palmBone = skel.Bones.FirstOrDefault(b => b.Id == OVRSkeleton.BoneId.Hand_WristRoot);


       /* if (palmBone != null)
        {

            var palmCapsule = palmBone.Transform.GetComponentInChildren<CapsuleCollider>();
            if (palmCapsule)
            {
                Debug.Log("--------------------PALMO PRESO--------------------" + palmCapsule);
                palmAnchor.position = palmCapsule.transform.position;
                palmAnchor.rotation = palmCapsule.transform.rotation;
                palmAnchor.SetParent(palmBone.Transform, true); // lo fai seguire il bone
                palmAnchor.localPosition += Vector3.up * 0.02f;
            }
        }
        else
        {
            Debug.Log("--------------------ERRORE PALMO--------------------------");
        }*/

        // adding a rigidbody
        var palmRb = palmAnchor.GetComponent<Rigidbody>();
        if (!palmRb) palmRb = palmAnchor.gameObject.AddComponent<Rigidbody>();
        palmRb.isKinematic = true;
        palmRb.useGravity = false;
        palmRb.interpolation = RigidbodyInterpolation.Interpolate;
        palmRb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
        
    }



    void FixedUpdate()
    {
        if (!palmAnchor) return;

        float dt = Time.fixedDeltaTime;

        // Linear Velocity (with smoothing)
        var rawVel = (palmAnchor.position - _lastPalmPos) / dt;
        _palmVelocity = Vector3.Lerp(rawVel, _palmVelocity, velSmoothing);

        // Angular velocity angolare (with smoothing)
        Quaternion dq = palmAnchor.rotation * Quaternion.Inverse(_lastPalmRot);
        dq.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;
        var rawAngVel = axis * (angleDeg * Mathf.Deg2Rad / dt);
        _palmAngularVelocity = Vector3.Lerp(rawAngVel, _palmAngularVelocity, angSmoothing);

        _lastPalmPos = palmAnchor.position;
        _lastPalmRot = palmAnchor.rotation;

        //Debug.Log("p" + PalmForward());
    }

    // ---  Events from Finger Colliders (capsules) ---

    public void OnFingerTouchEnter(Collider finger, Collider other, string fingerId = null)
    {
        if (!other.attachedRigidbody) return;

        var rb = other.attachedRigidbody;
        if (!IsGrabbable(rb.gameObject)) return;

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
        TryEndGrab(rb);
    }

    // --- Logica di presa/rilascio ---


    void TryBeginGrab(Rigidbody rb)
    {
        if (_isGrabbing) return;
        if (!_fingerContacts.ContainsKey(rb)) return;
        if (!palmAnchor) return;
        
        HashSet<string> fingersInContact = _fingerContacts[rb];

        bool thumbInContact = fingersInContact.Contains("Thumb");
        bool palmInContact = fingersInContact.Contains("Palm");
        bool hasOtherFinger = false;
        foreach (var f in fingersInContact) { if ((f != "Thumb") && (f != "Palm")) { hasOtherFinger = true; break; } }

        // forza gesto (isteresi gestita da grab/release thresholds) DA RIVEDERE!!!!!
        /*float pinchMax = Mathf.Max(
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Index),
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Thumb)
        );*/

        // PALM FACING: the object has to be in front of the palm
        Vector3 toObj = (rb.worldCenterOfMass - palmAnchor.position).normalized;
        float facing = Vector3.Dot(PalmForward(), toObj);
        bool palmFacingOk = facing >= palmFacingDotMin;


        // GRASPING CONDITION
        bool ok = (((thumbInContact || palmInContact) && hasOtherFinger) && palmFacingOk); 

        if (ok)
        {
            Debug.Log("----------BOOP-----------");
            //BeginGrab(rb);
        }

        // richiedi un numero minimo di frame consecutivi stabili in cui le condizioni persistono
        /*int neededFrames = Mathf.Max(debounceFrames, minSustainFrames);
        _framesStable = ok ? _framesStable + 1 : 0;

        if (_framesStable >= neededFrames)
        {
            BeginGrab(rb);
        }*/

    }


    void TryEndGrab(Rigidbody rb)
    {
        
        if (!_isGrabbing || rb != _currentBody) return;

        HashSet<string> fingersInContact = _fingerContacts.ContainsKey(rb) ? _fingerContacts[rb] : new HashSet<string>();
        bool thumbInContact = fingersInContact.Contains("Thumb");
        bool palmInContact = fingersInContact.Contains("Palm");
        bool hasOtherFinger = fingersInContact.Any(f => f != "Thumb" && f != "Palm");

        /*float pinchMax = Mathf.Max(
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Index),
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Thumb)
        );*/

        // RELEASE CONDITION
        bool release = (!(thumbInContact || palmInContact) || !hasOtherFinger); //|| (pinchMax <= releaseThreshold);

        //_framesStable = release ? _framesStable + 1 : 0;
        if (release)// && _framesStable >= debounceFrames)
        {
            Debug.Log("Try End");
            EndGrab();
        }
    }


    private void BeginGrab(Rigidbody rb)
    {
        if (!rb) return;
        Debug.Log("Grasping");

        // Cleaning preexisting joint on the palmAncor
        foreach (var j in palmAnchor.GetComponents<Joint>())
            Destroy(j);

        _currentBody = rb;

        // Zeroing the velocity on the attach 
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        //accelerazione ??
        rb.isKinematic = true;

        Vector3 relativePosition = palmAnchor.InverseTransformPoint(rb.transform.position);
        Quaternion relativeRotation = Quaternion.Inverse(palmAnchor.rotation) * rb.transform.rotation;

        rb.transform.SetParent(palmAnchor);
        rb.transform.localPosition = relativePosition;
        rb.transform.localRotation = relativeRotation;


        rb.solverIterations = Mathf.Max(rb.solverIterations, 12);
        rb.solverVelocityIterations = Mathf.Max(rb.solverVelocityIterations, 12);
        rb.maxAngularVelocity = Mathf.Max(rb.maxAngularVelocity, 50f);

        /*if (useFixedJoint)
        {
            var fj = palmAnchor.gameObject.AddComponent<FixedJoint>();
            fj.connectedBody = rb;
            fj.breakForce = breakForce;
            fj.breakTorque = breakTorque;
            _joint = fj; 

        }
        else //if i want to configure a different joint for soft interactions
        {*/
            /*var cj = palmAnchor.gameObject.AddComponent<ConfigurableJoint>();
            cj.connectedBody = rb;
            cj.breakForce = breakForce;
            cj.breakTorque = breakTorque;

            // Ancore esplicite
            cj.autoConfigureConnectedAnchor = false;
            cj.anchor = Vector3.zero;

            // >>> Punto di ancoraggio sull'oggetto = punto più vicino al palmo sui suoi colliders
            Vector3 closestOnRb = ComputeClosestPointOnColliders(rb, palmAnchor.position);
            cj.connectedAnchor = rb.transform.InverseTransformPoint(closestOnRb);

            if (softConfigurable)
            {
                cj.xMotion = ConfigurableJointMotion.Limited;
                cj.yMotion = ConfigurableJointMotion.Limited;
                cj.zMotion = ConfigurableJointMotion.Limited;
                cj.angularXMotion = ConfigurableJointMotion.Limited;
                cj.angularYMotion = ConfigurableJointMotion.Limited;
                cj.angularZMotion = ConfigurableJointMotion.Limited;

                var linLimit = new SoftJointLimit { limit = 0.003f }; // ~3 mm
                cj.linearLimit = linLimit;
                cj.linearLimitSpring = new SoftJointLimitSpring
                {
                    spring = positionSpring,
                    damper = positionDamper
                };

                cj.rotationDriveMode = RotationDriveMode.Slerp;
                cj.slerpDrive = new JointDrive
                {
                    positionSpring = rotationSpring,
                    positionDamper = rotationDamper,
                    maximumForce = Mathf.Infinity
                };
            }
            else
            {
                cj.xMotion = ConfigurableJointMotion.Locked;
                cj.yMotion = ConfigurableJointMotion.Locked;
                cj.zMotion = ConfigurableJointMotion.Locked;
                cj.angularXMotion = ConfigurableJointMotion.Locked;
                cj.angularYMotion = ConfigurableJointMotion.Locked;
                cj.angularZMotion = ConfigurableJointMotion.Locked;



                cj.rotationDriveMode = RotationDriveMode.Slerp;
                cj.slerpDrive = new JointDrive
                {
                    positionSpring = rotationSpring,
                    positionDamper = rotationDamper,
                    maximumForce = Mathf.Infinity
                };
            }

            // Projection per eliminare drift numerico
            cj.projectionMode = JointProjectionMode.PositionAndRotation;
            cj.projectionDistance = 0.005f;
            cj.projectionAngle = 3f;

            // Riduci l’influenza della massa dell’oggetto sulla mano
            cj.massScale = 1f;
            cj.connectedMassScale = 2.0f;

            // Evita oscillazioni iniziali aggressive
            cj.enablePreprocessing = false;*/
        //}

        _isGrabbing = true;
        Debug.Log("!!!!!!!! GRASPING OBJECT !!!!!!!");
    }

    void EndGrab()
    {
        if (_joint) Destroy(_joint);


        //Trasnfering the palm velocity to the object on release (Throw feature) 
        if (_currentBody && copyPalmVelocityOnRelease)
        {
            var v = _palmVelocity * throwMultiplier;
            var w = _palmAngularVelocity;
            if (v.magnitude > maxThrowSpeed) v = v.normalized * maxThrowSpeed;
            if (w.magnitude > maxThrowAngSpeed) w = w.normalized * maxThrowAngSpeed;

            _currentBody.velocity = v;
            _currentBody.angularVelocity = w;
            _currentBody.isKinematic = false;
            _currentBody.transform.SetParent(null); 

        }

        _currentBody = null;
        _fingerContacts.Clear();
        _isGrabbing = false;
        _framesStable = 0;
        Debug.Log("!!!!!!!! NOT GRASPING !!!!!!!");
    }

    void OnJointBreak(float bf)
    {
        _joint = null;
        _currentBody = null;
        _isGrabbing = false;
        _framesStable = 0;
    }

    void OnDisable()
    {
        if (_joint) Destroy(_joint);
        _currentBody = null;
        _isGrabbing = false;
        _framesStable = 0;
    }

    // --- Helpers ---

    private bool IsGrabbable(GameObject go)
    {
        if (grabbableTag.Length > 0 && !go.CompareTag(grabbableTag)) return false;
        return true;
    }

/*    ONLY USED WITH DIFFERENT JOINT (not fixed)
// Restituisce il punto (in world space) del collider dell'oggetto RB più vicino a "worldPoint".
// Ignora i collider disabilitati e quelli trigger. Se non trova nulla, usa il centro di massa.
private Vector3 ComputeClosestPointOnColliders(Rigidbody rb, Vector3 worldPoint)
{
    var colliders = rb.GetComponentsInChildren<Collider>();
    Vector3 best = rb.worldCenterOfMass;
    float bestSqr = float.PositiveInfinity;

    foreach (var c in colliders)
    {
        if (c == null || !c.enabled || c.isTrigger) continue;
        Vector3 p = c.ClosestPoint(worldPoint);
        float d2 = (p - worldPoint).sqrMagnitude;
        if (d2 < bestSqr)
        {
            bestSqr = d2;
            best = p;
        }
    }

    return best;
}*/
}