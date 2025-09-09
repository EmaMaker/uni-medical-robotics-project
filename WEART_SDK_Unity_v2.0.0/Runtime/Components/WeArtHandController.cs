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
        #endregion

        # region MEDICAL: variables+constants definition
        // -------------------- //
        // MEDICAL
        // Variable definition
        static VectorBuilder<double> V = Vector<double>.Build;
        static MatrixBuilder<double> M = Matrix<double>.Build;

        // Hands (use the same convention as CalibrationResult)
        static readonly int LEFT = 0;
        static readonly int RIGHT = 1;
        int CURRENT_HAND;
        string handString, handStringLong;

        // Fingers
        static readonly int THUMB = 0;
        static readonly int INDEX = 1;
        static readonly int MIDDLE = 2;
        static readonly int RING = 3;
        static readonly int PINKY = 4;
        static readonly int TOTAL_FINGERS = 5;
        static readonly int TOTAL_THIMBLES = 3;
        static readonly string[] finger_name = {"thumb", "index", "middle", "ring", "pinky"};
        // The objects depending on the last thimble (Colliders, Haptics, etc) use a slightly different name scheme
        static readonly string[] finger_name_end = {"Thumb", "Index", "Middle", "Annular", "Pinky"};

        // Five fingers, three thimbles each finger
        static Transform[,] finger_transform = new Transform[TOTAL_FINGERS,3];
        static double[,] finger_link_length = new double[TOTAL_FINGERS,3];
        static Vector3[,] finger_joint_initial_position = new Vector3[TOTAL_FINGERS,3];
        static Vector3[,] finger_joint_initial_rotation = new Vector3[TOTAL_FINGERS,3];
        static Vector<double>[] finger_robot_joint_angles = new Vector<double>[TOTAL_FINGERS];
        
        // Radii for arc of circle trajctories for IK
        static readonly double[] trajectory_arc_radii = {0.08d, 0.068d, 0.072d, 0.08d, 0.052};
        // Map finger to used index in _thimbles[i]
        // For TouchDiver G1, middle, ring and pinky all map to the middle
        static readonly int[] FINGER_TO_CLOSURE_INDEX = {0, 1, 2, 2, 2};

        // Define the minimum and maximum degrees for each finger (5) and thimble (3)
        // First index dictates the finger, second index the thimble
        static readonly double PI_HALF = MathF.PI * 0.5d;

        static double[,] FINGER_MAX_ANG = { { 30.0d * Mathf.Deg2Rad, 30.0d * Mathf.Deg2Rad, PI_HALF }, 
                                            { PI_HALF, PI_HALF, PI_HALF }, 
                                            { PI_HALF, PI_HALF, PI_HALF }, 
                                            { PI_HALF, PI_HALF, PI_HALF }, 
                                            { PI_HALF, PI_HALF, PI_HALF } };

        static double[,] FINGER_MIN_ANG = { { 0, 0, 0 },
                                            { 0, 0, 0 },
                                            { 0, 0, 0 },
                                            { 0, 0, 0 },
                                            { 0, 0, 0 } };

        // The arc of circle is parametrized as a curve by parameter t,[0,1]
        // where 0 is completely extended (q=0) and 1 completely curled (q=pi/2)
        // however this is made on a skeleton of the hand, and the visual result is not pleasant when looking
        // at a fully extended of fully curved hand
        // This parameter represent how to map the thimble curvature as received by the middeware [0, 1], to the parameter
        // t that parametrizes the trajectory and how much to "cut" so that the animation of the real hand never fully matches that
        // of the skeleton hand
        static double[] CLOSURE_TO_ARC_T_START = {0.0d, 0.0d, 0.0d, 0.0d, 0.0d};
        static double[] CLOSURE_TO_ARC_T_END = {0.0d, 0.0d, 0.0d, 0.0d, 0.0d};
        // To be properly calibrated with interface
        // static double[] CLOSURE_TO_ARC_T_START = {0.0d, 0.006d, 0.008d, 0.004d, 0.0d};
        // static double[] CLOSURE_TO_ARC_T_END = {0.0d, 0.13d, 0.16d, 0.16d, 0.11d};
        
        // the finger moves on the y-z plane
        static Vector<double>[] pExtended = new Vector<double>[TOTAL_FINGERS];
        static Vector<double>[] pCurled = new Vector<double>[TOTAL_FINGERS];
        
        // useful constants
        static Vector<double> ZEROS = V.DenseOfArray(new[] {0.0d,0.0d,0.0d});
        static Vector<double> ONES = V.DenseOfArray(new[] {1.0d,1.0d,1.0d});
        Matrix<double> I3 = M.DenseIdentity(3);
        static float TWO_PI = Mathf.PI * 2.0f;
        // END MEDICAL
        // -------------------- //
        #endregion

        #region Methods

        /// <summary>
        /// Initial set up
        /// </summary>
        private void Awake()
        {   
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
            
            // -------------------- //
            // MEDICAL: derive useful information from hand model
            // Navigate through the hierarcy to extract single GameObjects
            // WeArtHandController is attached to WEART>Hands>WeArtRightHand or WeArtLeftHand
            // WeArt*Hand>HandRig>HandRoot
            Transform handRoot = transform.Find("HandRig").Find("HandRoot");
            CURRENT_HAND = gameObject.name.Equals("WEARTRightHand") ? 1 : 0;
            handString  = gameObject.name.Equals("WEARTRightHand") ? "R" : "L";
            handStringLong  = gameObject.name.Equals("WEARTRightHand") ? "Right" : "Left";
            string debug_string = $"{handStringLong}  hand:\n";
            
            // Both hands use the suffix R for single joints, except the point with colliders, etc. that use proper Right and Left
            // HandRoot > DEF-hand.*
            Transform handDef = handRoot.Find("DEF-hand.R");

            // DEF-hand.* > ORG-palm.0n., n=1,2,3,4
            for(int j = 0; j < TOTAL_FINGERS; j++){
                int finger_model_index;
                string fdefname;
                // Both thumb and index are attached to ORG-palm.01
                // Also the thumb follows a naming scheme different from other fingers (namely missing the f_ prefix)
                if(j == THUMB) {
                    finger_model_index = 1;
                    fdefname = "DEF-thumb";
                }
                // The rest are attached to numbers in increasing order (02 middle, 03 ring, 04 pinky)
                else {
                    finger_model_index = j;
                    fdefname = "DEF-f_" + finger_name[j];
                }

                // ORG-palm.0n > DEF-f_finger.0n
                Transform org = handDef.Find("ORG-palm.0"+finger_model_index+".R");
                // Then each finger is made of three thimbles, ordered 01, 02, 03 nested
                finger_transform[j,0] = org.Find(fdefname+".01.R");
                finger_transform[j,1] = finger_transform[j,0].Find(fdefname+".02.R");
                finger_transform[j,2] = finger_transform[j,1].Find(fdefname+".03.R");

                // Now that each object has been found, extract useful information

                // Rotation + position
                for (int k = 0; k < TOTAL_THIMBLES; k++){
                    finger_joint_initial_rotation[j,k] = finger_transform[j,k].localRotation.eulerAngles;
                    finger_joint_initial_position[j,k] = finger_transform[j,k].position;
                    
                }
                //finger_robot_joint_angles[j] = V.DenseOfArray(new[]{(double)finger_joint_initial_rotation[j,0].x, (double)finger_joint_initial_rotation[j,1].x, (double)finger_joint_initial_rotation[j,2].x}) * (double)Mathf.Deg2Rad;
                finger_robot_joint_angles[j] = V.DenseOfArray(new[] { 0.0d, 0.0d, 0.0d });

                // Link lengths
                finger_link_length[j,0] = Vector3.Distance(finger_joint_initial_position[j,0], finger_joint_initial_position[j,1]);
                finger_link_length[j,1] = Vector3.Distance(finger_joint_initial_position[j,1], finger_joint_initial_position[j,2]);
                // This one sits exactly at the point of the finger
                Transform collider = finger_transform[j,2].Find(handStringLong + finger_name_end[j] + "ExplorationOrigin");
                finger_link_length[j,2] = Vector3.Distance(finger_joint_initial_position[j,2] , collider.position);

                double L1 = finger_link_length[j, 0];
                double L2 = finger_link_length[j, 1];
                double L3 = finger_link_length[j, 2];

                pExtended[j] = DK(j, V.DenseOfArray(new[] { FINGER_MIN_ANG[j, 0], FINGER_MIN_ANG[j, 1], FINGER_MIN_ANG[j, 2] }));
                pCurled[j] = DK(j, V.DenseOfArray(new[] { FINGER_MAX_ANG[j, 0], FINGER_MAX_ANG[j, 1], FINGER_MAX_ANG[j, 2] }));


                debug_string += $"{finger_name_end[j]}:\n\tLink lengths: L1: {finger_link_length[j,0], 3}, L2: {finger_link_length[j,1]}, L3: {finger_link_length[j,2]}" +
                                $"\n\tInitial rotations: L1: {finger_joint_initial_rotation[j,0]}, L2: {finger_joint_initial_rotation[j,1]}, L3: {finger_joint_initial_rotation[j,2]}\n";
            }
            WeArtLog.Log(debug_string);
            // END MEDICAL
            // -------------------- //
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

        # region MEDICAL: Trig functions

        // -------------------- //
        // MEDICAL: Trigonometric functions
        private double Sin(float f){ return (double) Mathf.Sin(f); }
        private double Sin(double f){ return (double) Mathf.Sin((float)f); }
        private double Cos(float f){ return (double) Mathf.Cos(f); }
        private double Cos(double f){ return (double) Mathf.Cos((float)f); }
        private double Sqrt(float f){ return (double) Mathf.Sqrt(f); }
        private double Sqrt(double f){ return (double) Mathf.Sqrt((float)f); }
        private double Atan2(float y, float x){ return (double) Mathf.Atan2(y,x); }
        private double Atan2(double y, double x){ return (double) Mathf.Atan2((float)y, (float)x); }
        // Unity does not handle switching from positive to negative angles very well, which however happens when taking joint angles from the controller
        // These functions bring angles in the [0, 360) degrees or [0, 2pi) radiants range
        private float fixAngleRad(float a) { return (a + TWO_PI) % TWO_PI; }
        private float fixAngleRad(double a){ return fixAngleRad((float)a); }
        private float fixAngleDeg(float a) { return (a + 360.0f) % 360.0f; }
        private float fixAngleDeg(double a){ return fixAngleDeg((float)a); }
        # endregion
        // END MEDICAL
        // -------------------- //

        # region MEDICAL: Robot manipulator functions
        // -------------------- //
        // MEDICAL: Robot manipulator functions
        // Direct kinematics
        private Vector<double> DK(int finger, Vector<double> q){
            double q1 = q.At(0);
            double q2 = q.At(1);
            double q3 = q.At(2);

            double L1 = finger_link_length[finger, 0];
            double L2 = finger_link_length[finger, 1];
            double L3 = finger_link_length[finger, 2];

            double x = (double) (L1*Sin(q1)+L2*Sin(q1+q2)+L3*Sin(q1+q2+q3));
            double y = (double) (L1*Cos(q1)+L2*Cos(q1+q2)+L3*Cos(q1+q2+q3));
            return V.DenseOfArray(new[] {x, y});
        }

        // Jacobian: task is end effector planar position (x, y)
        private Matrix<double> jacobian(int finger, Vector<double> q){
            double q1 = q.At(0);
            double q2 = q.At(1);
            double q3 = q.At(2);

            double L1 = finger_link_length[finger, 0];
            double L2 = finger_link_length[finger, 1];
            double L3 = finger_link_length[finger, 2];
            
            double[,] matrix = {
                {(double) (L1*Cos(q1) + L2*Cos(q1+q2) + L3*Cos(q1+q2+q3)), (double)(L2*Cos(q1+q2) + L3*Cos(q1+q2+q3)), (double)(L3*Cos(q1+q2+q3))},
                {(double) (-L1*Sin(q1) -L2*Sin(q1+q2) -L3*Sin(q1+q2+q3)), (double)(-L2*Sin(q1+q2) -L3*Sin(q1+q2+q3)), (double)(-L3*Sin(q1+q2+q3))},
            };
            return M.DenseOfArray(matrix);
        }

        // Velocity control. Use this function if using Euler integration
        private Vector<double> velocity_control_euler(int finger, Vector<double> q, double closure){
            Vector<double> fd = two_point_arc(finger, trajectory_arc_radii[finger], closure);
            Vector<double> dq0 = null_space_term(finger, q);

            Matrix<double> J = jacobian(finger, q);
            Matrix<double> J_ = J.PseudoInverse();
            Matrix<double> Proj = I3 - J_*J;
            Vector<double> f = DK(finger, q);

            return J_*50*(fd-f) + 50*Proj*dq0;
        }

        // Velocity control. Use this function if using Math.NET Numerics ODE solver
        // https://stackoverflow.com/questions/54999500/solving-a-system-of-ordinary-differential-equations-using-math-net-numerics
        // This is a delegate
        // https://stackoverflow.com/questions/3624731/what-is-func-how-and-when-is-it-used
        // https://learn.microsoft.com/en-us/dotnet/csharp/programming-guide/delegates/using-delegates
        // The first two parameters are types of arguments of the function (time, state) and the last is the return type
        // extra parameters (e.g. closure) are put as actual arguments to the function (I don't know if it's the right syntax, but it's the most clear to me and it works)
        private Func<double, Vector<double>, Vector<double>> velocity_control_ode(int finger, double cl) {
            // This is a lambda function
            return (t, q) =>
            {
                Vector<double> fd = two_point_arc(finger, trajectory_arc_radii[finger], cl);
                Vector<double> dq0 = null_space_term(finger, q);

                Matrix<double> J = jacobian(finger, q);
                Matrix<double> J_ = J.PseudoInverse();
                Matrix<double> Proj = I3 - J_*J;
                Vector<double> f = DK(finger, q);
                return J_*50*(fd-f) + 50*Proj*dq0;
            };
        }

        // Simulation function
        private Vector<double> sim(int finger, Vector<double> q, double closure){
            // Euler is fine, but movement in choppy even with 0.001 seconds of fixedDeltaTime
            // Also apparently the physics engine (thus fixedDeltaTime) is affected by interaction with unity editor, e.g. opening Project Settings
            // while game is running momentarly stops th physics engine and euler completely loses it
            // return q + velocity_control(q, closure)*Time.fixedUnscaledDeltaTime;

            // Solve ODE instead of relying on Euler. Better numerical accuracy, especially when changing direction (I hope)
            // System is time-invariant, so 0s to fixedDeltaTime is ok
            Vector<double>[] qnew = RungeKutta.SecondOrder(q, 0.0d, (double)Time.fixedDeltaTime, 20, velocity_control_ode(finger, closure));

            // Saturate joints to stay within limits
            // Particularly useful to filter out impulsive jumps from closure=0 to closure=1 caused by ill values sent by the interface
            // Saturating only the last sample of the simulation is ok, as it is the one used to animate the hand and as initial condition to the next simulation step
            double[] r1 = Enumerable.Range(0, FINGER_MIN_ANG.GetLength(1)).Select(x => FINGER_MIN_ANG[finger, x]).ToArray();
            double[] r2 = Enumerable.Range(0, FINGER_MAX_ANG.GetLength(1)).Select(x => FINGER_MAX_ANG[finger, x]).ToArray();
            Vector<double> limits_q_min = V.DenseOfArray(r1);
            Vector<double> limits_q_max = V.DenseOfArray(r2);

            return qnew[qnew.Length - 1].PointwiseMinimum(limits_q_max).PointwiseMaximum(limits_q_min);
        }

        // TODO: since we are adding a constant to each joint to set the zero position, make the
        // max joint slightly less than 
        private Vector<double> null_space_term(int finger, Vector<double> q){
            double q1 = q.At(0);
            double q2 = q.At(1);
            double q3 = q.At(2);

            //WeArtLog.Log($"{q1}, {q2}, {q3}");

            double[] dq0 = { ( FINGER_MAX_ANG[finger,0] - 2*q1 + FINGER_MIN_ANG[finger,0] ) / ( 6 * ( FINGER_MAX_ANG[finger, 0] - FINGER_MIN_ANG[finger,0] ) * (FINGER_MAX_ANG[finger, 0] - FINGER_MIN_ANG[finger, 0])  ),
                             ( FINGER_MAX_ANG[finger,1] - 2*q2 + FINGER_MIN_ANG[finger,1] ) / ( 6 * ( FINGER_MAX_ANG[finger, 1] - FINGER_MIN_ANG[finger,1] ) * (FINGER_MAX_ANG[finger, 1] - FINGER_MIN_ANG[finger, 1])  ),
                             ( FINGER_MAX_ANG[finger,2] - 2*q3 + FINGER_MIN_ANG[finger,2] ) / ( 6 * ( FINGER_MAX_ANG[finger, 2] - FINGER_MIN_ANG[finger,2] ) * (FINGER_MAX_ANG[finger, 2] - FINGER_MIN_ANG[finger, 2])  )
                            };

            return V.DenseOfArray(dq0);
        
        

        }

        // two point arc for trajectory
        // Finds the (portion of the) circle that passes between pExtended and pCurled and finds the exact position
        // at cl percentage between them
        private Vector<double> two_point_arc(int finger, double R, double cl){
            double xp1 = pExtended[finger].At(0);
            double yp1 = pExtended[finger].At(1);
            double xp2 = pCurled[finger].At(0);
            double yp2 = pCurled[finger].At(1);
            
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

            double t = CLOSURE_TO_ARC_T_START[finger] + (1-CLOSURE_TO_ARC_T_START[finger]-CLOSURE_TO_ARC_T_END[finger])*cl;
            double xArc = R*Sin(gamma_start + gamma*t) + xO;
            double yArc = R*Cos(gamma_start + gamma*t) + yO;
            
            return V.DenseOfArray(new[]{xArc, yArc});
        }

        // Simulate and animate fingers at physics simulation time (simulation time step is fixed, animation time step is not)
        // See: https://docs.unity3d.com/6000.1/Documentation/Manual/fixed-updates.html
        // For setting of initial conditions, see Awake()
        private void FixedUpdate(){

            for(int i = THUMB; i <= PINKY; i++){
                finger_robot_joint_angles[i] = sim(i, finger_robot_joint_angles[i], _thimbles[FINGER_TO_CLOSURE_INDEX[i]].Closure.Value);

                // Short hand                
                Vector<double> Q = finger_robot_joint_angles[i];
                Vector3 i_j1_initial = finger_joint_initial_rotation[i, 0];
                Vector3 i_j2_initial = finger_joint_initial_rotation[i, 1];
                Vector3 i_j3_initial = finger_joint_initial_rotation[i, 2];
                
                // Create new quaternions
                Quaternion quat = Quaternion.identity;
                Quaternion quat1 = Quaternion.identity;
                Quaternion quat2 = Quaternion.identity;
                // Assign them using Euler Angles. Keep initial y and z rotation. The joint of the manipulator moves around the x axis, positive clockwise
                if (i == THUMB) quat.eulerAngles = new Vector3(fixAngleDeg(i_j1_initial.x + Mathf.Rad2Deg * Q.At(0)), i_j1_initial.y, 50f * _thimbles[FINGER_TO_CLOSURE_INDEX[THUMB]].Abduction.Value );
                else quat.eulerAngles = new Vector3(fixAngleDeg(i_j1_initial.x + Mathf.Rad2Deg * Q.At(0)), i_j1_initial.y, i_j1_initial.z);
                
                quat1.eulerAngles = new Vector3(fixAngleDeg(i_j2_initial.x + Mathf.Rad2Deg * Q.At(1)), i_j2_initial.y, i_j2_initial.z);
                quat2.eulerAngles = new Vector3(fixAngleDeg(i_j3_initial.x + Mathf.Rad2Deg*Q.At(2)), i_j3_initial.y, i_j3_initial.z);
                // Set quaternions as rotation of each joint
                finger_transform[i, 0].localRotation = quat;
                finger_transform[i, 1].localRotation = quat1;
                finger_transform[i, 2].localRotation = quat2;
            }
        }
        // END MEDICAL
        // -------------------- //
        #endregion

        /// <summary>
        /// Method that takes care of finger animation
        /// </summary>
        private void AnimateFingers()
        {
            _graph.Evaluate();
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