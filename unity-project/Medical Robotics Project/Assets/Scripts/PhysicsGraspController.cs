using System.Collections.Generic;
using System.Linq;
using UnityEngine;

//da aggiungere a OVRHand

[RequireComponent(typeof(OVRHand))]
public class PhysicsGraspController : MonoBehaviour
{
    [Header("Refs")]
    public Transform palmAnchor;            // Empty sul palmo
    public OVRHand ovrHand;                 // assegnato o trovato in Reset()
    public HandColliderBuilder builder;     // opzionale: solo per riferimenti

    [Header("Filters")] //per come è lo script ora usa entrambi: più semplice il tag
    public string grabbableTag = "";    // metti nome nella stringa o se lasci vuoto -> ignora

    [Header("Thresholds (with hysteresis)")] //per evitare il flicker
    [Range(0, 1)] public float grabThreshold = 0.70f;
    [Range(0, 1)] public float releaseThreshold = 0.55f;
    public int debounceFrames = 2;          // anti-flicker

    [Header("Palm facing")]
    [Range(-1f, 1f)] public float palmFacingDotMin = 0.15f; // se >0 l'oggetto è davanti al palmo

    public int minSustainFrames = 1; //min frame necessary to start grasping

    // Se il forward del palmAnchor punta 'fuori dal palmo', va bene così.
    // Altrimenti, cambia qui l'asse usato per il test.
    private Vector3 PalmForward() => palmAnchor ? palmAnchor.forward : transform.forward;


    [Header("Joint Mode")]
    public bool useFixedJoint = true;       // ✔ semplice e robusto
    public bool softConfigurable = false;   // se false: Locked; se true: Limited + drive

    [Header("Joint Tuning (Configurable)")]
    public float positionSpring = 2000f;
    public float positionDamper = 80f;
    public float rotationSpring = 3000f;
    public float rotationDamper = 120f;
    public float breakForce = 1000f;
    public float breakTorque = 1000f;

    [Header("Release throw")]
    public bool copyPalmVelocityOnRelease = true;
    public float throwMultiplier = 1.0f;
    public float maxThrowSpeed = 12f;        // clamp velocità lineare
    public float maxThrowAngSpeed = 20f;     // clamp rad/s
    [Range(0f, 1f)] public float velSmoothing = 0.2f;
    [Range(0f, 1f)] public float angSmoothing = 0.2f;

    // stato velocità palmo
    private Vector3 _lastPalmPos;
    private Quaternion _lastPalmRot;
    private Vector3 _palmVelocity;
    private Vector3 _palmAngularVelocity;
    

    // contatti: RB -> set di dita (non colliders)
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
        if (!palmAnchor) Debug.LogError("[PhysicsGraspController] palmAnchor non assegnato!");
        if (!ovrHand) ovrHand = GetComponent<OVRHand>();

        var palmRb = palmAnchor.GetComponent<Rigidbody>();
        if (!palmRb) palmRb = palmAnchor.gameObject.AddComponent<Rigidbody>();
        palmRb.isKinematic = true;
        palmRb.useGravity = false;
        palmRb.interpolation = RigidbodyInterpolation.Interpolate;
        palmRb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;

        // setup stato per smoothing velocità palmo
        _lastPalmPos = palmAnchor.position;
        _lastPalmRot = palmAnchor.rotation;
        _palmVelocity = Vector3.zero;
        _palmAngularVelocity = Vector3.zero;

         /* collega automaticamente i reporter ai collider "HC_*"
        foreach (var col in GetComponentsInChildren<Collider>())
        {
            if (!col.name.StartsWith("HC_")) continue;
            var rep = col.gameObject.GetComponent<FingerContactController>();
            if (!rep) rep = col.gameObject.AddComponent<FingerContactController>();
            rep.controller = this;
        }*/

        // opzionale: evita che eventuali Joint residui rimangano attaccati al palmo
        foreach (var j in palmAnchor.GetComponents<Joint>())
            Destroy(j);
    }

    void FixedUpdate()
    {
        if (!palmAnchor) return;

        float dt = Time.fixedDeltaTime;

        // vel lineare (con smoothing)
        var rawVel = (palmAnchor.position - _lastPalmPos) / dt;
        _palmVelocity = Vector3.Lerp(rawVel, _palmVelocity, velSmoothing);

        // vel angolare (con smoothing)
        Quaternion dq = palmAnchor.rotation * Quaternion.Inverse(_lastPalmRot);
        dq.ToAngleAxis(out float angleDeg, out Vector3 axis);
        if (angleDeg > 180f) angleDeg -= 360f;
        var rawAngVel = axis * (angleDeg * Mathf.Deg2Rad / dt);
        _palmAngularVelocity = Vector3.Lerp(rawAngVel, _palmAngularVelocity, angSmoothing);

        _lastPalmPos = palmAnchor.position;
        _lastPalmRot = palmAnchor.rotation;
    }

    // --- Eventi dai colliders delle dita (FingerContactReporter) ---

    public void OnFingerTouchEnter(Collider finger, Collider other, string fingerId = null)
    {
        if (!other.attachedRigidbody) return;

        var rb = other.attachedRigidbody;
        if (!IsGrabbable(rb.gameObject)) return;

        if (!_fingerContacts.ContainsKey(rb)) _fingerContacts[rb] = new HashSet<string>();
        Debug.Log("collider finger: " + finger);
        Debug.Log(fingerId);
        _fingerContacts[rb].Add(FingerIdFromColliderName(finger));
        TryBeginGrab(rb);
    }

    public void OnFingerTouchExit(Collider finger, Collider other, string fingerId = null)
    {
        if (!other.attachedRigidbody) return;

        var rb = other.attachedRigidbody;
        if (_fingerContacts.ContainsKey(rb))
        {
            _fingerContacts[rb].Remove(FingerIdFromColliderName(finger));
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

        Debug.Log("Try Begin");
        
        HashSet<string> fingersInContact = _fingerContacts[rb];
        foreach (string s in fingersInContact)
        {
            Debug.Log("Fingers in contact:" + s);
        }
        
        bool thumbInContact = fingersInContact.Contains("Thumb");
        bool hasOtherFinger = false;
        foreach (var f in fingersInContact) { if (f != "Thumb") { hasOtherFinger = true; break; } }

        // forza gesto (isteresi gestita da grab/release thresholds)
        float pinchMax = Mathf.Max(
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Index),
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Thumb)
        );

        // PALM FACING: l'oggetto deve essere 'davanti' al palmo
        Vector3 toObj = (rb.worldCenterOfMass - palmAnchor.position).normalized;
        float facing = Vector3.Dot(PalmForward(), toObj);
        bool palmFacingOk = facing >= palmFacingDotMin;

        // condizione complessiva
        bool ok = thumbInContact && hasOtherFinger; //&& (pinchMax >= grabThreshold) && palmFacingOk;

        if (ok)
        {
            Debug.Log("----------OKAY-----------");
            BeginGrab(rb);
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
        bool hasOtherFinger = fingersInContact.Any(f => f != "Thumb");

        float pinchMax = Mathf.Max(
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Index),
            ovrHand.GetFingerPinchStrength(OVRHand.HandFinger.Thumb)
        );

        // condizione inversa: se manca il pollice o nessun altro dito, o pinch troppo basso → rilascio
        bool release = !thumbInContact || !hasOtherFinger || (pinchMax <= releaseThreshold);

        _framesStable = release ? _framesStable + 1 : 0;
        if (release && _framesStable >= debounceFrames)
        {
            Debug.Log("Try End");
            EndGrab();
        }
    }


    private void BeginGrab(Rigidbody rb)
    {
        if (!rb) return;
        Debug.Log("Grasping");

        // Pulisci eventuali joint preesistenti sul palmo
        foreach (var j in palmAnchor.GetComponents<Joint>())
            Destroy(j);

        _currentBody = rb;

        // Azzerare velocità all’attach (evita colpi elastici)
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Solver più “duro” in presa (meno jitter)
        rb.solverIterations = Mathf.Max(rb.solverIterations, 12);
        rb.solverVelocityIterations = Mathf.Max(rb.solverVelocityIterations, 12);
        rb.maxAngularVelocity = Mathf.Max(rb.maxAngularVelocity, 50f);

        if (useFixedJoint)
        {
            var fj = palmAnchor.gameObject.AddComponent<FixedJoint>();
            fj.connectedBody = rb;
            fj.breakForce = breakForce;
            fj.breakTorque = breakTorque;

        }
        else
        {
            var cj = palmAnchor.gameObject.AddComponent<ConfigurableJoint>();
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
            cj.enablePreprocessing = false;
        }

        _isGrabbing = true;
        Debug.Log("Grasping pronto ");
    }

    void EndGrab()
    {
        if (_joint) Destroy(_joint);

        if (_currentBody && copyPalmVelocityOnRelease)
        {
            var v = _palmVelocity * throwMultiplier;
            var w = _palmAngularVelocity;
            if (v.magnitude > maxThrowSpeed) v = v.normalized * maxThrowSpeed;
            if (w.magnitude > maxThrowAngSpeed) w = w.normalized * maxThrowAngSpeed;

            _currentBody.velocity = v;
            _currentBody.angularVelocity = w;
        }

        _currentBody = null;
        _isGrabbing = false;
        _framesStable = 0;
        Debug.Log("Grasping tolto ");
    }

    void OnJointBreak(float bf)
    {
        // sicurezza: se si rompe per forza eccessiva, ripulisci lo stato
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

    // Mappa nome collider -> dito (HC_Index1 -> "Index", HC_ThumbTip -> "Thumb", ecc.) HC_Hand_Ring1
    private string FingerIdFromColliderName(Collider col)
    {
        //if (!string.IsNullOrEmpty(fallback)) return fallback;

        string n = col.name.ToLower();
        Debug.Log("funzione" + n);
        if (n.Contains("thumb")) return "Thumb";
        else if (n.Contains("index")) return "Index";
        else if (n.Contains("middle")) return "Middle";
        else if (n.Contains("ring")) return "Ring";
        else if (n.Contains("pinky")) return "Pinky";
        return "Unknown";
    }
    
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
}
}
