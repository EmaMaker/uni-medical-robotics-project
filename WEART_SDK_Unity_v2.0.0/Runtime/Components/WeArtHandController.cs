using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Animations;
using UnityEngine.Playables;
using WeArt.Core;
using Texture = WeArt.Core.Texture;

using static WeArt.Components.WeArtTouchableObject;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using WeArt.Utils;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.OdeSolvers;
using System;

namespace WeArt.Components
{
    /// <summary>
    /// This component is able to animate a virtual hand using closure data coming from
    /// a set of <see cref="WeArtThimbleTrackingObject"/> components.
    /// </summary>
    [RequireComponent(typeof(Animator))]
    public class WeArtHandController : MonoBehaviour
    {
        #region Fields
        /// <summary>
        /// Defines the _openedHandState.
        /// </summary>
        
        [SerializeField]
        internal AnimationClip _openedHandState;

        /// <summary>
        /// Defines the _closedHandState.
        /// </summary>
        [SerializeField]
        internal AnimationClip _closedHandState;

        /// <summary>
        /// Defines the _abductionHandState.
        /// </summary>
        [SerializeField]
        internal AnimationClip _abductionHandState;

        /// <summary>
        /// Defines the finger animation mask.
        /// </summary>
        [SerializeField]
        internal AvatarMask _thumbMask, _indexMask, _middleMask, _ringMask, _pinkyMask;

        /// <summary>
        /// Defines the finger's thimble tracking component
        /// </summary>
        [SerializeField]
        internal WeArtThimbleTrackingObject _thumbThimbleTracking, _indexThimbleTracking, _middleThimbleTracking, _annularThimbleTracking, _pinkyThimbleTracking;

        /// <summary>
        /// Defines the finger's hapric object
        /// </summary>
        [SerializeField]
        internal WeArtHapticObject _thumbThimbleHaptic, _indexThimbleHaptic, _middleThimbleHaptic, _palmThimbleHaptic, _annularThimbleHaptic, _pinkyThimbleHaptic;

        /// <summary>
        /// Defines the visible hand's mesh renderer
        /// </summary>
        [SerializeField]
        internal SkinnedMeshRenderer _handSkinnedMeshRenderer;

        /// <summary>
        /// Defines the logo on the hand
        /// </summary>
        [SerializeField]
        internal GameObject _logoHandPanel;

        /// <summary>
        /// Defines the invisible material for the ghost hand
        /// </summary>
        [SerializeField]
        internal Material _ghostHandInvisibleMaterial;

        /// <summary>
        /// Defines the transparent material for the ghost hand
        /// </summary>
        [SerializeField]
        internal Material _ghostHandTransparentMaterial;

        /// <summary>
        /// If you use custom poses, you have to add WeArtGraspPose.cs
        /// </summary>
        [SerializeField]
        internal bool _useCustomPoses;

        /// <summary>
        /// Defines the _animator.
        /// </summary>
        private Animator _animator;

        /// <summary>
        /// Defines the _fingers.
        /// </summary>
        private AvatarMask[] _fingers;

        /// <summary>
        /// Defines the _thimbles.
        /// </summary>
        private WeArtThimbleTrackingObject[] _thimbles;

        /// <summary>
        /// Defines the player graph
        /// </summary>
        private PlayableGraph _graph;

        /// <summary>
        /// Defines the animation layers
        /// </summary>
        private AnimationLayerMixerPlayable[] _fingersMixers;

        /// <summary>
        /// Returns a read only list of the animation layers
        /// </summary>
        public IReadOnlyList<AnimationLayerMixerPlayable> FingersMixers => _fingersMixers.ToList();

        /// <summary>
        /// Define the hand side
        /// </summary>
        [SerializeField]
        internal HandSide _handSide;

        // Variables for finger animation
        private float _fingersAnimationSpeed = 10f;
        private float _thumbAnimationSpeed = 20f;
        private float _fingersSlideSpeed = 1.5f;
        private float _extraFingerSpeed = 10f;
        private float _safeUnlockFrames = 0.2f;
        private float _slowFingerAnimationTime;
        private float _slowFingerAnimationTimeMax = 1f;

        /// <summary>
        /// Defines the grasping system component
        /// </summary>
        private WeArtHandGraspingSystem _graspingSystem;

        /// <summary>
        /// Defines the surface exploration component
        /// </summary>
        private WeArtHandSurfaceExploration _surfaceExploration;

        /// <summary>
        /// Defines the opposite hand in the scene
        /// </summary>
        private WeArtHandController _otherHand;

        // MEDICAL
        // Transforms, Math.NET Numerics
        static Transform i_t, i_t1, i_t2, i_t3;
        static Vector3 i_j1_initial, i_j2_initial, i_j3_initial;
        static VectorBuilder<double> V = Vector<double>.Build;
        static MatrixBuilder<double> M = Matrix<double>.Build;
        static Vector<double> Q;

        #endregion

        #region Methods

        /// <summary>
        /// Initial set up
        /// </summary>
        private void Awake()
        {
            // Find transforms of finger parts
            i_t = transform.Find("HandRig").Find("HandRoot").Find("DEF-hand.R").Find("ORG-palm.01.R").Find("DEF-f_index.01.R");
            i_t1 = i_t.Find("DEF-f_index.02.R");
            i_t2 = i_t1.Find("DEF-f_index.03.R");

            // Set initial Q to angles as set in the model
            i_j1_initial = i_t.localRotation.eulerAngles;
            i_j2_initial = i_t1.localRotation.eulerAngles;
            i_j3_initial = i_t2.localRotation.eulerAngles;
            WeArtLog.Log("J1: " + i_j1_initial + ", J2: " + i_j2_initial + ", J3: " + i_j3_initial);
            Q = V.DenseOfArray(new[]{(double)i_j1_initial.x, (double)i_j2_initial.x, (double)i_j3_initial.x}) * (double)Mathf.Deg2Rad;

            // Setup components
            _graspingSystem = GetComponent<WeArtHandGraspingSystem>();
            _surfaceExploration = GetComponent<WeArtHandSurfaceExploration>();

            if (_graspingSystem == null)
            {
                WeArtLog.Log("Does Not Have The Grasping System Component", LogType.Error);
            }

            // Setup animation components
            _animator = GetComponent<Animator>();
            _fingers = new AvatarMask[] { _thumbMask, _indexMask, _middleMask, _ringMask, _pinkyMask };
            _thimbles = new WeArtThimbleTrackingObject[] {
                _thumbThimbleTracking,
                _indexThimbleTracking,
                _middleThimbleTracking,
                _annularThimbleTracking,
                _pinkyThimbleTracking
            };

            _thumbThimbleHaptic.SetIsUsedByController(true);
            _thumbThimbleHaptic.HandController = this;
            _indexThimbleHaptic.SetIsUsedByController(true);
            _indexThimbleHaptic.HandController = this;
            _middleThimbleHaptic.SetIsUsedByController(true);
            _middleThimbleHaptic.HandController = this;
            _annularThimbleHaptic.SetIsUsedByController(true);
            _annularThimbleHaptic.HandController = this;
            _pinkyThimbleHaptic.SetIsUsedByController(true);
            _pinkyThimbleHaptic.HandController = this;
            _palmThimbleHaptic.SetIsUsedByController(true);
            _palmThimbleHaptic.HandController = this;

            WeArtHandController[] components = GameObject.FindObjectsOfType<WeArtHandController>();
            foreach (var component in components)
            {
                if (component._handSide != _handSide)
                {
                    _otherHand = component;
                }
            }
 
        }

        /// <summary>
        /// The OnEnable.
        /// </summary>
        private void OnEnable()
        {
            _graph = PlayableGraph.Create(nameof(WeArtHandController));

            var fingersLayerMixer = AnimationLayerMixerPlayable.Create(_graph, _fingers.Length);
            _fingersMixers = new AnimationLayerMixerPlayable[_fingers.Length];

            for (uint i = 0; i < _fingers.Length; i++)
            {
                var fingerMixer = AnimationLayerMixerPlayable.Create(_graph, 3);
                _graph.Connect(AnimationClipPlayable.Create(_graph, _openedHandState), 0, fingerMixer, 0);
                _graph.Connect(AnimationClipPlayable.Create(_graph, _closedHandState), 0, fingerMixer, 1);
                _graph.Connect(AnimationClipPlayable.Create(_graph, _abductionHandState), 0, fingerMixer, 2);

                fingerMixer.SetLayerAdditive(0, false);
                fingerMixer.SetLayerMaskFromAvatarMask(0, _fingers[i]);
                fingerMixer.SetInputWeight(0, 1);
                fingerMixer.SetInputWeight(1, 0);
                _fingersMixers[i] = fingerMixer;

                fingersLayerMixer.SetLayerAdditive(i, false);
                fingersLayerMixer.SetLayerMaskFromAvatarMask(i, _fingers[i]);
                _graph.Connect(fingerMixer, 0, fingersLayerMixer, (int)i);
                fingersLayerMixer.SetInputWeight((int)i, 1);
            }

            var handMixer = AnimationMixerPlayable.Create(_graph, 2);
            _graph.Connect(fingersLayerMixer, 0, handMixer, 0);
            handMixer.SetInputWeight(0, 1);
            var playableOutput = AnimationPlayableOutput.Create(_graph, nameof(WeArtHandController), _animator);
            playableOutput.SetSourcePlayable(handMixer);
            _graph.SetTimeUpdateMode(DirectorUpdateMode.GameTime);
            _graph.Play();

            // Subscribe custom finger closure behaviour during the grasp
            _graspingSystem.OnGraspingEvent += UpdateFingerClosure;
        }

        /// MEDICAL: Check this
        /// <summary>
        /// Handle the behaviour of all fingers during the grasp
        /// </summary>
        private void UpdateFingerClosure(HandSide hand, GameObject gameObject)
        {
            if (_useCustomPoses)
            {
                // In this case you have to use WeArtGraspPose on your touchable object to handle the fingers poses
                if (TryGetCustomPosesFromTouchable(gameObject, out var customPoses))
                {
                    StopAllCoroutines();

                    for (int i = 0; i < customPoses.fingersClosure.Length; i++)
                    {
                        var weight = customPoses.fingersClosure[i];

                        StartCoroutine(LerpPoses(_fingersMixers[i], 0, 1 - weight, customPoses.lerpTime));
                        StartCoroutine(LerpPoses(_fingersMixers[i], 1, weight, customPoses.lerpTime));
                    }
                }
            }
        }

        /// <summary>
        /// Get the hand controller's grasping system
        /// </summary>
        /// <returns></returns>
        public WeArtHandGraspingSystem GetGraspingSystem()
        { return _graspingSystem; }

        /// <summary>
        /// Get the other Hand Controller
        /// </summary>
        /// <returns></returns>
        public WeArtHandController GetOtherHand()
        { return _otherHand; }
        /// <summary>
        /// Get time in seconds of the time of animation for moving the fingers slowly after release
        /// </summary>
        /// <returns></returns>
        public float GetSafeUnlockFrames()
        { return _safeUnlockFrames; }

        /// <summary>
        /// Set time left to move the fingers slowly to avoid glitch collisions
        /// </summary>
        /// <param name="time"></param>
        public void SetSlowFingerAnimationTime(float time)
        { _slowFingerAnimationTime = time; }

        public float GetSlowFingerAnimationTimeMax()
        { return _slowFingerAnimationTimeMax; }


        /// <summary>
        /// Getr the invisible material
        /// </summary>
        /// <returns></returns>
        public Material GetGhostHandInvisibleMaterial()
        {
            return _ghostHandInvisibleMaterial;
        }

        /// <summary>
        /// Get the transparent material
        /// </summary>
        /// <returns></returns>
        public Material GetGhostHandTransparentMaterial()
        {
            return _ghostHandTransparentMaterial;
        }

        /// <summary>
        /// Get the hand side in flag format
        /// </summary>
        /// <returns></returns>
        public HandSideFlags GetHandSideFlag()
        {
            if (_handSide == HandSide.Right)
                return HandSideFlags.Right;
            else
                return HandSideFlags.Left;
        }

        /// <summary>
        /// Debug method
        /// </summary>
        private void OnDrawGizmos()
        {
        }

        /// <summary>
        /// The Update.
        /// </summary>
        private void Update()
        {
            if (_graspingSystem.GraspingState == GraspingState.Grabbed)
            {
                if(_graspingSystem.GetGraspedObject() != null)
                {
                    if(_graspingSystem.GetGraspedObject().GraspingType == GraspingType.Snap)
                    {
                        _graspingSystem.UpdateGraspingSystem();
                        return;
                    }
                }
            }

                AnimateFingers();

                _surfaceExploration.SurfaceExplorationCheck();

                _graspingSystem.UpdateGraspingSystem();
            
        }

        static double L1 = 0.0435d;
        static double L2 = 0.0333d;
        static double L3 = 0.0204d;
        static float TWO_PI = Mathf.PI * 2.0f;
        
        // The arc of circle is parametrized as a curve by parameter t,[0,1]
        // where 0 is completely extended (q=0) and 1 completely curled (q=pi/2)
        // however this is made on a skeleton of the hand, and the visual result is not pleasant when looking
        // at a fully extended of fully curved hand
        // This parameter represent how to map the thimble curvature as received by the middeware [0, 1], to the parameter
        // t that parametrizes the trajectory and how much to "cut" so that the animation of the real hand never fully matches that
        // of the skeleton hand
        // e.g. CLOSURE_TO_ARC = 0.1d means closure=0 corresponds to t=0.1 and closure=1 corresponds to t=1
        static double CLOSURE_TO_ARC_T_START = 0.006d;
        static double CLOSURE_TO_ARC_T_END = 0.1d;
        // the finger moves on the y-z plane
        // static Vector<float> pExtended = new Vector2(0, L1+L2+L3);
        // static Vector2 pCurled = new Vector2(L1-L3, -L2);
        static Vector<double> pExtended = V.DenseOfArray(new[] {0.0d, L1+L2+L3});
        static Vector<double> pCurled = V.DenseOfArray(new[] {L1-L3, -L2});
        
        static Vector<double> ZEROS = V.DenseOfArray(new[] {0.0d,0.0d,0.0d});
        static Vector<double> ONES = V.DenseOfArray(new[] {1.0d,1.0d,1.0d});
        Matrix<double> I3 = M.DenseIdentity(3);
        /* Measurements found by:
        {
            // find child of child .. of child thimbles via concatenating transforms
            Transform i_t = transform.Find("HandRig").Find("HandRoot").Find("DEF-hand.R").Find("ORG-palm.01.R").Find("DEF-f_index.01.R");
            Transform i_t1 = i_t.Find("DEF-f_index.02.R");
            Transform i_t2 = i_t1.Find("DEF-f_index.03.R");
            Transform i_t3 = i_t2.Find("RightIndexExplorationOrigin");

            float L1 = Vector3.Distance(i_t.position, i_t1.position);
            float L2 = Vector3.Distance(i_t1.position, i_t2.position);
            float L3 = Vector3.Distance(i_t2.position, i_t3.position);
            WeArtLog.Log("Link1: " + L1 + ", Link2: " + L2 + ", Link3: " + L3);
        }
        */

        private double Sin(float f){
            return (double) Mathf.Sin(f);
        }
        private double Sin(double f){
            return (double) Mathf.Sin((float)f);
        }
        private double Cos(float f){
            return (double) Mathf.Cos(f);
        }
        private double Cos(double f){
            return (double) Mathf.Cos((float)f);
        }
        private double Sqrt(float f){
            return (double) Mathf.Sqrt(f);
        }
        private double Sqrt(double f){
            return (double) Mathf.Sqrt((float)f);
        }
        private double Atan2(float y, float x){
            return (double) Mathf.Atan2(y,x);
        }
        private double Atan2(double y, double x){
            return (double) Mathf.Atan2((float)y, (float)x);
        }
        
        private float fixAngleRad(float a) { 
            return (a + TWO_PI) % TWO_PI;
        }
        private float fixAngleRad(double a){ return fixAngleRad((float)a); }
        private float fixAngleDeg(float a) { 
            return (a + 360.0f) % 360.0f;
        }
        private float fixAngleDeg(double a){ return fixAngleDeg((float)a); }
        /* ---- MEDICAL ---- */
        // Direct kinematics
        private Vector<double> DK(Vector<double> q){
            double q1 = q.At(0);
            double q2 = q.At(1);
            double q3 = q.At(2);
            double x = (double) (L1*Sin(q1)+L2*Sin(q1+q2)+L3*Sin(q1+q2+q3));
            double y = (double) (L1*Cos(q1)+L2*Cos(q1+q2)+L3*Cos(q1+q2+q3));
            return V.DenseOfArray(new[] {x, y});
        }

        private Matrix<double> jacobian(Vector<double> q){
            double q1 = q.At(0);
            double q2 = q.At(1);
            double q3 = q.At(2);
            
            double[,] matrix = {
                {(double) (L1*Cos(q1) + L2*Cos(q1+q2) + L3*Cos(q1+q2+q3)), (double)(L2*Cos(q1+q2) + L3*Cos(q1+q2+q3)), (double)(L3*Cos(q1+q2+q3))},
                {(double) (-L1*Sin(q1) -L2*Sin(q1+q2) -L3*Sin(q1+q2+q3)), (double)(-L2*Sin(q1+q2) -L3*Sin(q1+q2+q3)), (double)(-L3*Sin(q1+q2+q3))},
            };
            return M.DenseOfArray(matrix);
        }

        private Vector<double> velocity_control_euler(Vector<double> q, double closure){
            Vector<double> fd = two_point_arc(0.07f, closure);
            Vector<double> dq0 = null_space_term(q);

            Matrix<double> J = jacobian(q);
            Matrix<double> J_ = J.PseudoInverse();
            Matrix<double> Proj = I3 - J_*J;
            Vector<double> f = DK(q);
            // return J_*50*(fd-f) + 50*Proj*dq0;
            return J_*50*(fd-f) + 50*Proj*dq0;
        }

        // https://stackoverflow.com/questions/54999500/solving-a-system-of-ordinary-differential-equations-using-math-net-numerics
        // This is a delegate
        // https://stackoverflow.com/questions/3624731/what-is-func-how-and-when-is-it-used
        // https://learn.microsoft.com/en-us/dotnet/csharp/programming-guide/delegates/using-delegates
        // The first two parameters are types of arguments of the function (time, state) and the last is the return type
        // extra parameters (e.g. closure) as actual arguments to the function (I don't know if it's the right syntax, but it's the most clear to me)
        private Func<double, Vector<double>, Vector<double>> velocity_control_ode(double cl) {
            // This is a lambda function
            return (t, q) =>
            {
                Vector<double> fd = two_point_arc(0.07f, cl);
                Vector<double> dq0 = null_space_term(q);

                Matrix<double> J = jacobian(q);
                Matrix<double> J_ = J.PseudoInverse();
                Matrix<double> Proj = I3 - J_*J;
                Vector<double> f = DK(q);
                return J_*50*(fd-f) + 50*Proj*dq0;
            };
            // Vector<double> fd = two_point_arc(0.07f, closure);
            // Vector<double> dq0 = null_space_term(q);
        }

        private Vector<double> sim(Vector<double> q, double closure){
            // Euler is fine, but movement in choppy even with 0.001 seconds of fixedDeltaTime
            // Also apparently the physics engine (thus fixedDeltaTime) is affected by interaction with unity editor, e.g. opening Project Settings
            // while game is running momentarly stops th physics engine and euler completely loses it
            // return q + velocity_control(q, closure)*Time.fixedUnscaledDeltaTime;

            // Solve ODE instead of relying on Euler. Better numerical accuracy, especially when changing direction (I hope)
            // System is time-invariant, so 0s to fixedDeltaTime is ok
            Vector<double>[] qnew = RungeKutta.SecondOrder(q, 0.0d, (double)Time.fixedDeltaTime, 20, velocity_control_ode(closure));
            return qnew[qnew.Length - 1];
        }

        // TODO: since we are adding a constant to each joint to set the zero position, make the
        // max joint slightly less than 
        private Vector<double> null_space_term(Vector<double> q){
            double q1 = q.At(0);
            double q2 = q.At(1);
            double q3 = q.At(2);
            // double dq01 = (4*q.x - Mathf.PI) / (3*Mathf.PI*Mathf.PI);
            return -V.DenseOfArray(new[] {4.0d*q1 - Mathf.PI, 4.0d*q2 - Mathf.PI, 4.0d*q3 - Mathf.PI}) / (3.0d*Mathf.PI*Mathf.PI);
        }

        // two point arc for trajectory
        // Finds the (portion of the) circle that passes between pExtended and pCurled and finds the exact position
        // at cl percentage between them
        // TODO: Optimizi this stuff to compute just one time, or rewrite to support positions as arguments
        // for future use with multiple fingers
        private Vector<double> two_point_arc(double R, double cl){
            double xp1 = pExtended.At(0);
            double yp1 = pExtended.At(1);
            double xp2 = pCurled.At(0);
            double yp2 = pCurled.At(1);
            
            double D = yp2*yp2-yp1*yp1+xp2*xp2-xp1*xp1;
            double E = D/(2*(xp2-xp1));
            double C = (yp1-yp2)/(xp2-xp1);
            
            double a = C*C + 1;
            double b = 2*(E*C - E*C*xp1 - yp1);
            double c = E*E - R*R + xp1*xp1 + yp1*yp1 -2*xp1*E;
            
            double yO = (-b - Sqrt(b*b-4*a*c))/(2*a);
            double xO = E+yO*C;
        
            double gamma_start = Atan2(xp1-xO, yp1-yO);
            double gamma_end = Atan2(xp2-xO, yp2-yO);
            double gamma = gamma_end-gamma_start;

            // double t = cl;
            double t = CLOSURE_TO_ARC_T_START + (1-CLOSURE_TO_ARC_T_START-CLOSURE_TO_ARC_T_END)*cl;
            double xArc = R*Sin(gamma_start +gamma*t) + xO;
            double yArc = R*Cos(gamma_start +gamma*t) + yO;
            
            return V.DenseOfArray(new[]{xArc, yArc});
        }

        // MEDICAL: Move fingers at physics simulation time (simulation time step is fixed)
        // See: https://docs.unity3d.com/6000.1/Documentation/Manual/fixed-updates.html
        // For setting of initial conditions, see Awake()
        bool first = true;
        private void FixedUpdate(){          
            
            if(Time.time <= 2.5) return; // Give some time for calibration to finish
            // if(first){
            //     Q = V.DenseOfArray(new[]{(double)i_j1_initial.x, (double)i_j2_initial.x, (double)i_j3_initial.x}) * (double)Mathf.Deg2Rad;
            //     first = false;
            // }
            
            // Move joints between 0 and pi/2 manually
            // Q = ONES * CLOSURE * 0.5d * Mathf.PI;

            // Compute closure at runtime as a sine function (start extended)
            // CLOSURE = 0.5d*(1.0d - Cos(0.8d*Time.unscaledTime));
            // Q = sim(Q, CLOSURE);

            // Take closure from interface, data in [0,1] as parsed by middleware
            Q = sim(Q, _thimbles[1].Closure.Value);
            
            // TODO: Hard limit angles to [initial+0, initial+pi/2], otherwise it results in some unnatural manipulator-like
            // movements, especially with the finger slightly folding the opposite way when extending

            // Sets the rotation to given angles for each joint
            // For each one, also add the initial value, as it sets the "zero" position
            // Fix the angles in [0, 360], Unity does not handle well jumps between positive and negative angles and considers angles in [0,360] anyway
            // Also mantain inital rotation in other axes
            Quaternion quat = Quaternion.identity;
            Quaternion quat1 = Quaternion.identity;
            Quaternion quat2 = Quaternion.identity;

            // quat.eulerAngles = new Vector3(fixAngleDeg(i_j1_initial.x + Mathf.Rad2Deg*Q.At(0)), 0, 0);
            // quat1.eulerAngles = new Vector3(fixAngleDeg(i_j2_initial.x + Mathf.Rad2Deg*Q.At(1)), 0, 0);
            // quat2.eulerAngles = new Vector3(fixAngleDeg(i_j3_initial.x + Mathf.Rad2Deg*Q.At(2)), 0, 0);

            quat.eulerAngles = new Vector3(fixAngleDeg(i_j1_initial.x + Mathf.Rad2Deg*Q.At(0)), i_j1_initial.y, i_j1_initial.z);
            quat1.eulerAngles = new Vector3(fixAngleDeg(i_j2_initial.x + Mathf.Rad2Deg*Q.At(1)), i_j2_initial.y, i_j2_initial.z);
            quat2.eulerAngles = new Vector3(fixAngleDeg(i_j3_initial.x + Mathf.Rad2Deg*Q.At(2)), i_j3_initial.y, i_j3_initial.z);
            
            // Hard limit "display" angles (transforms), but not manipulator angles (Q)
            // quat.eulerAngles = Vector3.Max(i_j1_initial, Vector3.Min(quat.eulerAngles, Vector3.one*110.0f));
            // quat1.eulerAngles = Vector3.Max(i_j2_initial, Vector3.Min(quat1.eulerAngles, Vector3.one*110.0f));
            // quat2.eulerAngles = Vector3.Max(i_j3_initial, Vector3.Min(quat2.eulerAngles, Vector3.one*110.0f));
            
            i_t.localRotation = quat;
            i_t1.localRotation = quat1;
            i_t2.localRotation = quat2;
        }

        // MEDICAL: Check this
        /// <summary>
        /// Method that takes care of finger animation
        /// </summary>
        private void AnimateFingers()
        {
            // _graph.Evaluate();

            // if (_useCustomPoses && _graspingSystem.GraspingState == GraspingState.Grabbed)
            // {
            //     // In this case the fingers not follow the tracking but are driven by WeArtGraspPose
            //     // The behaviour is called in this script in -> UpdateFingerClousure
            // }
            // else // Otherwise fingers behaviour works as always
            // {
            
            // WeArtLog.Log($"Closure: " + cl + "\nQ1: " + Q.At(0) + ", Q2: " + Q.At(1) + ", Q3: " + Q.At(2));

                // GameObject[] components = gameObject.GetComponents<GameObject>();
                // foreach (GameObject c in components){
                //     WeArtLog.Log(c.name);
                // }
                // for (int i = 0; i < gameObject.GetComponentCount(); i++){
                //     WeArtLog.Log(gameObject.GetComponentAtIndex(i));
                // }

                //t.Translate(0,-0.001f*Mathf.Sign(((float)Mathf.Sin(0.01f*Time.frameCount))),0);
                // Rotates by x amount of degrees
                //i_t.Rotate(45 + 45*(float)Mathf.Sin(0.000001f*Time.unscaledTime),0,0);
                
                


            //     //----

            //     for (int i = 0; i < _fingers.Length; i++)
            //     {
            //         if (!_thimbles[i].IsBlocked)
            //         {
            //             bool isGettingCloseToGrasp = false;

            //             if (i == 0 && _graspingSystem.ThumbGraspCheckTouchables.Count > 0 && _thimbles[i].Closure.Value > _fingersMixers[i].GetInputWeight(1))
            //                 isGettingCloseToGrasp = true; // Finger is getting close to an object that it can grab and slows the speed in order to ensure a perfect position on the touchable object

            //             if (i == 1 && _graspingSystem.IndexGraspCheckTouchables.Count > 0 && _thimbles[i].Closure.Value > _fingersMixers[i].GetInputWeight(1))
            //                 isGettingCloseToGrasp = true;

            //             if (i == 2 && _graspingSystem.MiddleGraspCheckTouchables.Count > 0 && _thimbles[i].Closure.Value > _fingersMixers[i].GetInputWeight(1))
            //                 isGettingCloseToGrasp = true;

            //             if (WeArtController.Instance._deviceGeneration == DeviceGeneration.TD_Pro)
            //             {
            //                 if (i == 3 && _graspingSystem.AnnularGraspCheckTouchables.Count > 0 && _thimbles[i].Closure.Value > _fingersMixers[i].GetInputWeight(1))
            //                     isGettingCloseToGrasp = true;

            //                 if (i == 4 && _graspingSystem.PinkyGraspCheckTouchables.Count > 0 && _thimbles[i].Closure.Value > _fingersMixers[i].GetInputWeight(1))
            //                     isGettingCloseToGrasp = true;
            //             }

            //             float weight;
            //             if (!isGettingCloseToGrasp)
            //             {
            //                 weight = _thimbles[i].Closure.Value;
            //             }
            //             else // If in proximity, move slower in order to avoid clipping through colliders at high movement speed per frame
            //             {
            //                 weight = Mathf.Lerp(_fingersMixers[i].GetInputWeight(1), _thimbles[i].Closure.Value,
            //                    Time.deltaTime * (_thimbles[i].SafeUnblockSeconds > 0 ? _fingersSlideSpeed : _fingersAnimationSpeed));
            //             }

            //             if (_slowFingerAnimationTime > 0)
            //             {
            //                 weight = Mathf.Lerp(_fingersMixers[i].GetInputWeight(1), _thimbles[i].Closure.Value,
            //                    Time.deltaTime * _fingersSlideSpeed * _extraFingerSpeed);
            //             }

            //             if (i > 2 && WeArtController.Instance._deviceGeneration == DeviceGeneration.TD)
            //             {
            //                 _fingersMixers[i].SetInputWeight(0, 1 - _fingersMixers[2].GetInputWeight(1));
            //                 _fingersMixers[i].SetInputWeight(1, _fingersMixers[2].GetInputWeight(1));
            //             }
            //             else
            //             {
            //                 _fingersMixers[i].SetInputWeight(0, 1 - weight);
            //                 _fingersMixers[i].SetInputWeight(1, weight);
            //             }

            //             // Thumb has an extra field called abduction that allows the finger to move up and down (non closing motion)
            //             if (_thimbles[i].ActuationPoint == ActuationPoint.Thumb)
            //             {
            //                 float abduction;
            //                 if (!isGettingCloseToGrasp)
            //                 {
            //                     abduction = _thimbles[i].Abduction.Value;
            //                 }
            //                 else
            //                 {
            //                     abduction = Mathf.Lerp(_fingersMixers[i].GetInputWeight(2), _thimbles[i].Abduction.Value,
            //                     Time.deltaTime * (_thimbles[i].SafeUnblockSeconds > 0 ? _fingersSlideSpeed : _fingersAnimationSpeed));
            //                 }

            //                 if (_slowFingerAnimationTime > 0)
            //                 {
            //                     abduction = Mathf.Lerp(_fingersMixers[i].GetInputWeight(2), _thimbles[i].Abduction.Value,
            //                     Time.deltaTime * (_thimbles[i].SafeUnblockSeconds > 0 ? _fingersSlideSpeed * _extraFingerSpeed : _thumbAnimationSpeed));
            //                 }

            //                 _fingersMixers[i].SetInputWeight(2, abduction);
            //             }

            //             if (_thimbles[i].SafeUnblockSeconds > 0)
            //                 _thimbles[i].SafeUnblockSeconds -= Time.deltaTime;
            //         }
            //     }
            // }


            // Slow finger animation countdown
            // if (_slowFingerAnimationTime > 0)
            //     _slowFingerAnimationTime -= Time.deltaTime;
        }

        public void SetFingerAnimation(EasyGraspData data)
        {
            //Thumb
            _fingersMixers[0].SetInputWeight(0, 1 - data.thumbClosure);
            _fingersMixers[0].SetInputWeight(1, data.thumbClosure);
            _fingersMixers[0].SetInputWeight(2, data.thumbAbduction);

            //Index
            _fingersMixers[1].SetInputWeight(0, 1 - data.indexClosure);
            _fingersMixers[1].SetInputWeight(1, data.indexClosure);

            //Middle
            _fingersMixers[2].SetInputWeight(0, 1 - data.middleClosure);
            _fingersMixers[2].SetInputWeight(1, data.middleClosure);

            //Annular
            _fingersMixers[3].SetInputWeight(0, 1 - data.annularClosure);
            _fingersMixers[3].SetInputWeight(1, data.annularClosure);

            //Pinky
            _fingersMixers[4].SetInputWeight(0, 1 - data.pinkyClosure);
            _fingersMixers[4].SetInputWeight(1, data.pinkyClosure);
        }

        /// <summary>
        /// The OnDisable
        /// </summary>
        private void OnDisable()
        {
            // Disable animation
            _graph.Destroy();

            _graspingSystem.OnGraspingEvent -= UpdateFingerClosure;
        }


        /// <summary>
        /// Calibration manager signaling its success
        /// </summary>
        public void CalibrationSuccessful()
        {
            _thumbThimbleHaptic.RemoveAllEffects();
            _indexThimbleHaptic.RemoveAllEffects();
            _middleThimbleHaptic.RemoveAllEffects();

            if(WeArtController.Instance._deviceGeneration == DeviceGeneration.TD_Pro)
            {
                _annularThimbleHaptic.RemoveAllEffects();
                _pinkyThimbleHaptic.RemoveAllEffects();
                _palmThimbleHaptic.RemoveAllEffects();
            }
        }

        /// <summary>
        /// Enables/Disables thimble haptic objects at the hand.
        /// </summary>
        /// <param name="enable"></param>
        public void EnableThimbleHaptic(bool enable)
        {
            _thumbThimbleHaptic.EnablingActuating(enable);
            _indexThimbleHaptic.EnablingActuating(enable);
            _middleThimbleHaptic.EnablingActuating(enable);

            if(WeArtController.Instance._deviceGeneration == DeviceGeneration.TD_Pro)
            {
                _annularThimbleHaptic.EnablingActuating(enable);
                _pinkyThimbleHaptic.EnablingActuating(enable);
                _palmThimbleHaptic.EnablingActuating(enable);
            }
        }

        /// <summary>
        /// Gets WeArtHapticObject related to actuation point.
        /// </summary>
        /// <param name="actuationPoint"></param>
        /// <returns></returns>
        public WeArtHapticObject GetHapticObject(ActuationPoint actuationPoint)
        {
            switch (actuationPoint)
            {
                case ActuationPoint.Thumb:
                    return _thumbThimbleHaptic;
                case ActuationPoint.Index:
                    return _indexThimbleHaptic;
                case ActuationPoint.Middle:
                    return _middleThimbleHaptic;
                case ActuationPoint.Palm:
                    return _palmThimbleHaptic;
                case ActuationPoint.Annular:
                    return _annularThimbleHaptic;
                case ActuationPoint.Pinky:
                    return _pinkyThimbleHaptic;
                default:
                    return null;
            }
        }

        /// <summary>
        /// Gets WeArtThimbleTrackingObject related to actuation point.
        /// </summary>
        /// <param name="actuationPoint"></param>
        /// <returns></returns>
        public WeArtThimbleTrackingObject GetThimbleTrackingObject(ActuationPoint actuationPoint)
        {
            switch (actuationPoint)
            {
                case ActuationPoint.Thumb:
                    return _thumbThimbleTracking;
                case ActuationPoint.Index:
                    return _indexThimbleTracking;
                case ActuationPoint.Middle:
                    return _middleThimbleTracking;
                case ActuationPoint.Palm:
                    return null;
                case ActuationPoint.Annular:
                    return _annularThimbleTracking;
                case ActuationPoint.Pinky:
                    return _pinkyThimbleTracking;
                default:
                    return null;
            }
        }

        /// <summary>
        /// Try to find a touchable object component on the collider
        /// </summary>
        /// <param name="collider">The collider<see cref="Collider"/>.</param>
        /// <param name="touchableObject">The touchableObject<see cref="WeArtTouchableObject"/>.</param>
        /// <returns>The <see cref="bool"/>.</returns>
        private static bool TryGetTouchableObjectFromCollider(Collider collider, out WeArtTouchableObject touchableObject)
        {
            touchableObject = collider.gameObject.GetComponent<WeArtTouchableObject>();
            return touchableObject != null;
        }

        /// <summary>
        /// Try to get WeArtGraspPose script from current touchable object
        /// </summary>
        /// <param name="gameObject">Current grasped object</param>
        /// <param name="customPoses">Output of this result</param>
        /// <returns></returns>
        private static bool TryGetCustomPosesFromTouchable(GameObject gameObject, out WeArtGraspPose customPoses)
        {
            customPoses = gameObject.GetComponent<WeArtGraspPose>();
            return customPoses != null;
        }


        #endregion

        #region Coroutines

        /// <summary>
        /// Interpolate the finger pose during grasp when using WeArtGraspPose.cs
        /// </summary>
        /// <param name="finger"></param>
        /// <param name="inputIndex"></param>
        /// <param name="closure"></param>
        /// <param name="lerpTime"></param>
        /// <returns></returns>
        private IEnumerator LerpPoses(AnimationLayerMixerPlayable finger, int inputIndex, float closure, float lerpTime)
        {
            float t = 0f;
            float to = finger.GetInputWeight(inputIndex);

            while (t < lerpTime)
            {
                float lerp;
                lerp = Mathf.Lerp(to, closure, t / lerpTime);
                t += Time.deltaTime;

                finger.SetInputWeight(inputIndex, lerp);
                yield return null;
            }
        }
        #endregion

    }
}