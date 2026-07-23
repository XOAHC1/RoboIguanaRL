using System;
using Hinge = VehicleComponents.Actuators.Hinge;
using Unity.MLAgents.Actuators;
using UnityEngine;

namespace RoboIguanaRL
{
    /// <summary>
    /// CPG based controller for RoboIguana. This class is intended to be controlled by an instance of <code>RoboIguanaAgentRL</code>.
    /// </summary>
    public class RoboIguanaCPGController: MonoBehaviour
    {
        // =========================================================
        // ACTUATORS
        // =========================================================

        /// <summary>
        /// Hinge for hip yaw movement.
        /// </summary>
        [Header("Hip yaw actuator:")]
        public Hinge hyFL, hyFR, hyRL, hyRR;

        /// <summary>
        /// Hinge for hip pith movement.
        /// </summary>
        [Header("Hip pitch actuator:")]
        public Hinge hpFL, hpFR, hpRL, hpRR;

        /// <summary>
        /// Hinge for knee movement.
        /// </summary>
        [Header("Knee actuator:")]
        public Hinge kFL, kFR, kRL, kRR;

        
        /// <summary>
        /// Hinge for sipne movement.
        /// </summary>
        [Header("Spine actuator:")]
        public Hinge spinePitch, spineYaw;

        /// <summary>
        /// Force point to simulate buoyancy control module.
        /// </summary>
        [Header("Buoyancy Module")]
        public SimpleForcePoint BuoyancyForcePoint;


        // =========================================================
        // PHYSICAL JOINTS
        // =========================================================

        /// <summary>
        /// <c>ArticulationBody</c> for yaw movement in the hip.
        /// </summary>
        [Header("Hip yaw link:")]
        public ArticulationBody hyFL_Link, hyFR_Link, hyRL_Link, hyRR_Link;

        /// <summary>
        /// <c>ArticulationBody</c> for pitch movement in the hip.
        /// </summary>
        [Header("Hip pitch link:")]
        public ArticulationBody hpFL_Link, hpFR_Link, hpRL_Link, hpRR_Link;

        /// <summary>
        /// <c>ArticulationBody</c> for movement in the knee.
        /// </summary>
        [Header("Knee link:")]
        public ArticulationBody kFL_Link, kFR_Link, kRL_Link, kRR_Link;

        /// <summary>
        /// <c>ArticulationBody</c> for movement in the spine.
        /// </summary>
        [Header("Spine link:")]
        public ArticulationBody spine_link_pitch, spine_link_yaw;


        // =========================================================
        // Helper Modules
        // =========================================================

        /// <summary>
        /// Helper object to manage the robot's tail.
        /// </summary>
        private TailManager Tail;


        // =========================================================
        // LEG GEOMETRY
        // =========================================================

        // 0.116, 0.09, 0.172, 0.2
        /// <summary>
        /// Parameters for leg geometry
        /// </summary>
        [Header("Leg geometry (meters)")]        
        public float a;        
        public float b;        
        public float c;
        public float d;


        // =========================================================
        // Trajectory parameters
        // =========================================================

        [Header("Trajectory Parameters")]
        /// <summary>
        /// Step length of the foot trajectory.
        /// </summary>
        public float dStep; // 0.15f;

        /// <summary>
        /// Ground clearance during swing phase.
        /// </summary>
        public float gC; // 0.04f;

        /// <summary>
        /// Ground penetration during stance phase.
        /// </summary>
        public float gP; // 0.03f;

        /// <summary>
        /// Height of the robot body.
        /// </summary>
        public float h; // 0.18f;

        /// <summary>
        /// Maximum rotation range for spine in degrees.
        /// </summary>
        public float spineRangePitch, spineRangeYaw; // 15, 10;

        /// <summary>
        /// Maximum range for sway and yaw of the tail in degrees.
        /// </summary>
        public float tailSwayRange, tailYawRange; // 20, 40;

        [Header("Buoyancy Module Limits")]
        public float maxBuoyancy; // 2.25f [N]
        public float maxBuoyancyShift; // 0.3 [N/dt]

        [Header("Training parameters")]
        /// <summary>
        /// Convergence rate for amplitude shifts (literature notation: a).
        /// </summary>
        public float convergence; // 1f;

        /// <summary>
        /// Time step for CPG updates in seconds.
        /// </summary>
        private float TimeStep;

        /// <summary>
        /// Initial foot positions for each leg (FL, FR, RL, RR).
        /// <remarks>
        /// Only needed for retreiving initial CPG states.
        /// </remarks>
        /// </summary>
        private readonly Vector3[] initialFootPositions = {
            new Vector3(0.075f, -0.18f, 0f),
            new Vector3(-0.075f, -0.18f, -0f),
            new Vector3(-0.075f, -0.18f, 0f),
            new Vector3(0.075f, -0.18f, -0f)
        };


        // =========================================================
        // Arrays for collecticve Access
        // =========================================================

        /// <summary>
        /// Current joint angles.
        /// </summary>
        private float[] 
            yaws = new float[4],
            hips = new float[4], 
            knees = new float[4];

        /// <summary>
        /// Array of <c>ArticulationBody</c> to faciltate access.
        /// </summary>
        private ArticulationBody[] hipYawLinks, hipPitchLinks, kneeLinks, spineLinks;

        /// <summary>
        /// Array of <c>Hinge</c> to faciltate access.
        /// </summary>
        private Hinge[] hipYawHinges, hipPitchHinges, kneeHinges, spineHinges;

        /// <summary>
        /// Array for easier access.
        /// </summary>
        private float[] spineRanges;

        // =========================================================
        // CPG PARAMETERS       Leg Order: FL, FR, RL, RR
        // =========================================================

        /// <summary>
        /// Initial CPG phases for legs and spine (Theta). Leg order: FL, FR, RL, RR, Spine pitch, Spine yaw.
        /// </summary>
        private readonly float[] initialPhases = {0f, Mathf.PI, Mathf.PI, 0f, 0f, 0f};
        /// <summary>
        /// Initial phase shift rates for legs and spine.
        /// </summary>
        private readonly float[] initialPhaseShifts = {0f, 0f, 0f, 0f, 0f, 0f};

        /// <summary>
        /// Initial amplitude values for legs (4) and spine (2).
        /// </summary>
        private readonly float[] initialAmplitudes = {0.5f, 0.5f, 0.5f, 0.5f, 0f, 1f};
        /// <summary>
        /// Initial amplitude shift rates for legs and spine.
        /// </summary>
        private readonly float[] initialAmplitudeShifts = {0f, 0f, 0f, 0f, 0f, 0f};
        /// <summary>
        /// Initial second derivative of amplitude shifts for legs and spine.
        /// </summary>
        private readonly float[] initialAmplitudeShifts2 = {0f, 0f, 0f, 0f, 0f, 0f};

        /// <summary>
        /// Initial orientation offsets for foot trajectories (Phi). Leg order: FL, FR, RL, RR.
        /// </summary>
        private readonly float[] initialOrientationOffsets = {0f, 0f, 0f, 0f};
        /// <summary>
        /// Initial orientation offset shift rates for legs. Controlled via psy.
        /// </summary>
        private readonly float[] initialOrientationOffsetShifts = {0f, 0f, 0f, 0f};

        /// <summary>
        /// Current CPG phases (Theta).
        /// </summary>
        private float[] Phases;
        /// <summary>
        /// Current CPG phase shift rates (Theta'). Controlled via omega.
        /// </summary>
        private float[] PhaseShifts;

        /// <summary>
        /// Current intrinsic amplitudes (r).
        /// </summary>
        private float[] Amplitudes;
        /// <summary>
        /// Current amplitude shift rates (r').
        /// </summary>
        private float[] AmplitudeShifts;
        /// <summary>
        /// Current second derivative of amplitude shifts (r''). Controlled via mu.
        /// </summary>
        private float[] AmplitudeShifts2;

        /// <summary>
        /// Current orientation offsets for foot trajectories (Phi).
        /// </summary>
        private float[] OrientationOffsets;
        /// <summary>
        /// Current orientation offset shift rates (Phi'). Controlled via psy.
        /// </summary>
        private float[] OrientationOffsetShifts;

        private Vector3 Buoyancy;
        private float BuoyancyShift;

        private int[] TailChanges = new int[3];

        // =====================================
        // Action indices
        // =====================================

        /// <summary>
        /// Starting index in Agent's actions for actions regarding this aspect.
        /// </summary>
        private int ActionIdxAmp, ActionIdxOrientation, ActionIdxBuoyancy;


        // =========================================================
        // Communication with RL Agent
        // =========================================================

        /// <summary>
        /// Gets a copy of the current CPG phases.
        /// </summary>
        /// <returns>A copy of the Phases array.</returns>
        public float[] GetPhases() { return (float[])Phases.Clone();}
        public float[] GetPhaseShifts() {return (float[])PhaseShifts.Clone();}

        /// <summary>
        /// Gets a copy of the current CPG amplitudes.
        /// </summary>
        /// <returns>A copy of the Amplitudes array.</returns>
        public float[] GetAmplitudes() { return (float[])Amplitudes.Clone();}
        public float[] GetAmplitudeShifts() { return (float[])AmplitudeShifts.Clone();}

        /// <summary>
        /// Gets a copy of the current orientation offsets.
        /// </summary>
        /// <returns>A copy of the OrientationOffsets array.</returns>
        public float[] GetOrientationOffsets() { return (float[])OrientationOffsets.Clone();}
        public float[] GetOrientationOffsetShifts() { return (float[])OrientationOffsetShifts.Clone();}

        /// <summary>
        /// Get current force added by the buoyancy module.
        /// </summary>
        /// <returns></returns>
        public float GetBuoyancy() {return Buoyancy.y;}
        public float GetBuoyancyShift() {return BuoyancyShift;} 

        /// <summary>
        /// Returns State of the tail.
        /// </summary>
        /// <returns></returns>
        public float[] GetTailState() {return new float[] {Tail.frequency, Tail.swayAmplitude, Tail.yawAmplitude, Tail.phase};}


        // =========================================================
        // Internal function
        // =========================================================

        /// <summary>
        /// Initializes the CPG controller by reseting all parameters and initializing all joints.
        /// </summary>
        public void Initialize()
        {
            Debug.Log("Initialze CPG");
            // get components
            Tail = GetComponent<TailManager>();
            Tail.Initialize();

            TimeStep = Time.fixedDeltaTime;

            // group joints for easier access.
            hipYawLinks = new ArticulationBody[] {hyFL_Link, hyFR_Link, hyRL_Link, hyRR_Link};
            hipYawHinges = new Hinge[] {hyFL, hyFR, hyRL, hyRR};
            hipPitchLinks = new ArticulationBody[] {hpFL_Link, hpFR_Link, hpRL_Link, hpRR_Link};
            hipPitchHinges = new Hinge[] {hpFL, hpFR, hpRL, hpRR};
            kneeLinks = new ArticulationBody[] {kFL_Link, kFR_Link, kRL_Link, kRR_Link};
            kneeHinges = new Hinge[] {kFL, kFR, kRL, kRR};

            spineLinks = new ArticulationBody[] {spine_link_pitch, spine_link_yaw};
            spineHinges = new Hinge[] {spinePitch, spineYaw};

            spineRanges = new float[] {spineRangePitch, spineRangeYaw};

            // Set up internal states
            InitializeAllJoints();
            ResetCPG();
            ResetBuoyancy();

            // prepare indices for updates
            ActionIdxAmp = Phases.Length;
            ActionIdxOrientation = ActionIdxAmp + Amplitudes.Length;
            ActionIdxBuoyancy = ActionIdxOrientation + OrientationOffsets.Length;

            // Debug.Log("CPG Ready");
        }

        /// <summary>
        /// Connects articulation bodies and hinges. Sets all joints to their initial pose.
        /// </summary>
        private void InitializeAllJoints()
        {
            // Initialize joint positions based on CPG parameters
            for (int i = 0; i<4; i++)
            {
                int left = (i %2 == 0) ? 1: -1;
                (float x, float y, float z) p = GetFootPosition(initialPhases[i], initialAmplitudes[i], initialOrientationOffsets[i], left);
                (float yaw, float hip, float knee) = InverseKinematics(p);

                InitialiseJoint(hipYawLinks[i], hipYawHinges[i], yaw);
                InitialiseJoint(hipPitchLinks[i], hipPitchHinges[i], hip);
                InitialiseJoint(kneeLinks[i], kneeHinges[i], knee);
            }

            // Spine and tail
            for (int i = 0; i < spineRanges.Length; i++)
            {
                InitialiseJoint(spineLinks[i], spineHinges[i], GetSpineAngle(initialPhases[i+4], initialAmplitudes[i+4]) * spineRanges[i]);
            }
        }

        /// <summary>
        /// Connects articulation body and hinge. Sets Jont to given pose.
        /// </summary>
        /// <param name="ab"></param>
        /// <param name="h"></param>
        /// <param name="angleRad"></param> <summary>
        private void InitialiseJoint(ArticulationBody ab, Hinge h, float angleRad)
        {
            if (ab == null) return;

            ab.jointPosition = new ArticulationReducedSpace(angleRad);
            var drive = ab.xDrive;
            drive.target = angleRad;
            ab.xDrive = drive;

            if (h != null)
                h.SetAngle(angleRad);
        }

        /// <summary>
        /// Sets all CPG parameters to their initial values. Updates pose to apply new parameters.
        /// </summary>
        public void Reset()
        {
            ResetBuoyancy();
            ResetCPG();
            UpdatePose();
            Tail.Reset();
        }

        /// <summary>
        /// Resets CPG-related arrays to their initial values.
        /// </summary>
        private void ResetCPG()
        {
            Phases = (float[])initialPhases.Clone();
            PhaseShifts = (float[])initialPhaseShifts.Clone();
            Amplitudes = (float[])initialAmplitudes.Clone();
            AmplitudeShifts = (float[])initialAmplitudeShifts.Clone();
            AmplitudeShifts2 = (float[])initialAmplitudeShifts2.Clone();
            OrientationOffsets = (float[])initialOrientationOffsets.Clone();
            OrientationOffsetShifts = (float[])initialOrientationOffsetShifts.Clone();
        }

        /// <summary>
        /// Resets tail control parameters to initial values.
        /// </summary>
        private void ResetBuoyancy()
        {
            Buoyancy = new Vector3(0f, 0f, 0f);
            BuoyancyShift = 0f;
        }

        /// <summary>
        /// Update for each time step. Handles CPG oscillations and calls pose update.
        /// </summary>
        public void FixedUpdate()
        {
            // Debug.Log("Fixed Update CPG");
            UpdateCPG();
            UpdatePose();
            UpdateBuoyancy();
        }

        /// <summary>
        /// Progress CPG by one step.
        /// </summary>
        private void UpdateCPG()
        {
            for (int i = 0; i < Phases.Length; i++)
            {
                Phases[i] = (Phases[i] + PhaseShifts[i] * TimeStep) % (2 * Mathf.PI);
            }

            for (int i = 0; i < AmplitudeShifts2.Length; i++)
            {
                Amplitudes[i] += AmplitudeShifts[i] * TimeStep;
                AmplitudeShifts[i] += AmplitudeShifts2[i] * TimeStep;
            }

            for (int i = 0; i < OrientationOffsets.Length; i++)
            {
                OrientationOffsets[i] = (OrientationOffsets[i] + OrientationOffsetShifts[i] * TimeStep) % (2 * Mathf.PI);
            }    
        }

        /// <summary>
        /// Apply current CPG parameters to robot pose.
        /// </summary>
        private void UpdatePose()
        {
            // update limb positions
            for (int i = 0; i < 4; i++) {
                // Apply an offset to the foot position based on the side of the foot.
                int left = (i % 2 == 0) ? 1: -1;
                (float x, float y, float z) p = GetFootPosition(Phases[i], Amplitudes[i], OrientationOffsets[i], left);
                (yaws[i], hips[i], knees[i]) = InverseKinematics(p);
            }
            ApplyAngles(yaws, hips, knees);

            // update spine position
            float[] spineAngles = new float[2];
            for (int i = 0; i < 2; i++) {
                spineAngles[i] = GetSpineAngle(Phases[i + 4], Amplitudes[i + 4]);
            }
            ApplySpineAngle(spineAngles);
        }

        /// <summary>
        /// Applies actions received from the RL agent to the CPG parameters.
        /// <remark>
        ///     Assuming the action space is structured as follows:
        ///         0-6: Phase shifts (legs: 4, spine: 2, tail: 1).
        ///        7-15: Amplitude shifts (legs: 4, spine: 2, tail: 2).
        ///       16-18: trajectory rotation shifts (legs: 4).
        ///          19: Phase lag shift for the tail. 
        /// </remark>
        /// </summary>
        /// <param name="actions">Action buffers containing continuous actions for phase shifts, amplitude shifts, and orientation offsets.</param>
        public void ApplyActions(ActionBuffers actions)
        {
            // Debug.Log("Applying Actions");
            // Apply the actions received from the RL agent to the CPG parameters
            ActionSegment<float> continuous = actions.ContinuousActions;
            ActionSegment<int> discrete = actions.DiscreteActions;

            // update phase and amplitude for all joints
            for (int i = 0; i < Phases.Length; i++)
            {
                // adapt phase shifts
                PhaseShifts[i] = continuous[i];
            }

            for (int i = 0; i < Amplitudes.Length; i++) {

                // adapt second derivative of amplitude 
                AmplitudeShifts2[i] =  convergence * ((convergence / 4) * (continuous[i + ActionIdxAmp] - Amplitudes[i]) - AmplitudeShifts[i]);
            }

            // update trajectory rotation shifts
            for (int i = 0; i < OrientationOffsetShifts.Length; i++)
            {
                OrientationOffsetShifts[i] = continuous[i + ActionIdxOrientation];
            }

            // Buoyancy Module
            BuoyancyShift = continuous[ActionIdxBuoyancy] * maxBuoyancyShift;

            // Tail Parameters
            for (int i = 0; i < TailChanges.Length; i++)
            {
                TailChanges[i] = discrete[i] - 1;
            }

            Tail.UpdateParameters(TailChanges);
        }

        /// <summary>
        /// Handles time step for buoyancy control module. Applies corresponding force.
        /// </summary>
        private void UpdateBuoyancy()
        {
            Buoyancy.y = Mathf.Clamp(Buoyancy.y + BuoyancyShift * TimeStep, 0, maxBuoyancy) * 1.8f;
            
            BuoyancyForcePoint.ApplyWorldForce(Buoyancy);
        }

        // =========================================================
        // CPG translation
        // =========================================================

        /// <summary>
        /// Calculates the foot position based on CPG state parameters.
        /// </summary>
        /// <param name="phase">The current phase of the CPG oscillator.</param>
        /// <param name="amplitude">The amplitude of the foot trajectory.</param>
        /// <param name="orientationOffset">The orientation offset for the foot trajectory.</param>
        /// <returns>A tuple containing the x, y, z coordinates of the foot position.</returns>
        private (float x, float y, float z) GetFootPosition(float phase, float amplitude, float orientationOffset, int left)
        {
            float x = -dStep * (amplitude - 1.0f) * MathF.Sin(phase) * MathF.Cos(orientationOffset);
            float y  = -h + (MathF.Cos(phase) > 0.0f ? gC : gP) * MathF.Cos(phase);
            float z = -dStep * (amplitude - 1.0f) * MathF.Sin(phase) * MathF.Sin(orientationOffset) + left * 0.25f;

            return (x, y, z);
        }

        /// <summary>
        /// Calculates spine and tail angles based on CPG state parameters.
        /// </summary>
        /// <param name="phase">The current phase of the CPG oscillator.</param>
        /// <param name="amplitude">The amplitude of the spine trajectory.</param>
        /// <returns>The angle for the spine/tail joint.</returns>
        private float GetSpineAngle(float phase, float amplitude)
        {
            return MathF.Sin(phase) * amplitude;
        }

        // =========================================================
        // Inverse Kinematics (Copied from original Project)
        // =========================================================

        /// <summary>
        /// Wrapper for <c>InverseKinematics((float x, float y, float z) p)</c> to handle <c>Vector3</c> inputs.
        /// </summary>
        /// <param name="p">The foot position as a Vector3.</param>
        /// <returns>A tuple containing yaw, hip, and knee angles.</returns>
        private (float yaw, float hip, float knee) InverseKinematics(Vector3 p)
        {
            return InverseKinematics((p.x, p.y, p.z));
        }

        /// <summary>
        /// Calculates joint angles from a foot position given as x, y, z coordinates.
        /// </summary>
        /// <param name="p">A tuple containing the x, y, z coordinates of the foot position.</param>
        /// <returns>A tuple containing yaw, hip, and knee angles.</returns>
        private (float yaw, float hip, float knee) InverseKinematics((float x, float y, float z) p)
        {
            float yaw;
            float hip;
            float knee;

            float x = p.x;
            float y = p.y;
            float z = p.z;

            float r2 = x * x + z * z;
            float Lmag = Mathf.Sqrt(Mathf.Max(0f, r2 - b * b));

            yaw = Mathf.Atan2(z, -x);

            float Xp = Lmag - a;
            float Yp = -y;

            float D = (Xp * Xp + Yp * Yp - c * c - d * d) / (2f * c * d);
            D = Mathf.Clamp(D, -1f, 1f);

            knee = Mathf.Atan2(Mathf.Sqrt(1 - D * D), D);

            hip = Mathf.Atan2(Yp, Xp) -
                    Mathf.Atan2(d * Mathf.Sin(knee),
                                c + d * Mathf.Cos(knee));

            return (yaw, hip, knee);
        }

        /// <summary>
        /// Applies the calculated joint angles to all limbs.
        /// </summary>
        /// <param name="yaw">Array of yaw angles for each leg (FL, FR, RL, RR).</param>
        /// <param name="hip">Array of hip angles for each leg (FL, FR, RL, RR).</param>
        /// <param name="knee">Array of knee angles for each leg (FL, FR, RL, RR).</param>
        private void ApplyAngles(
                float[] yaw, float[] hip, float[] knee)
        {
            // Debug.Log("Applying Angles");
            // Leg Order: FL, FR, RL, RR
            hyFL.SetAngle(yaw[0]);
            hyFR.SetAngle(yaw[1]);
            hyRL.SetAngle(yaw[2]);
            hyRR.SetAngle(yaw[3]);

            hpFL.SetAngle(hip[0]);
            hpFR.SetAngle(hip[1]);
            hpRL.SetAngle(hip[2]);
            hpRR.SetAngle(hip[3]);

            kFL.SetAngle(knee[0]);
            kFR.SetAngle(knee[1]);
            kRL.SetAngle(knee[2]);
            kRR.SetAngle(knee[3]);
        }

        /// <summary>
        /// Applies the calculated angles to all spine joints.
        /// </summary>
        /// <param name="angles">Array containing spine angles.</param>
        private void ApplySpineAngle(float[] angles)
        {
            for (int i = 0; i < angles.Length; i++) spineHinges[i].SetAngle(angles[i] * spineRanges[i]);
            // spinePitch.SetAngle(angles[0] * spineRangePitch);
            // spineYaw.SetAngle(angles[1] * spineRangeYaw);
        }


        // =========================================================
        // Debugging functions for individual use
        // =========================================================

        /// <summary>
        /// Recovers CPG parameters to match the initial foot positions based on initial phases.
        /// </summary>
        private void FindStartingCPGState()
        {
            Debug.Log("Recovered Parameters after initialization:");

            for (int i = 0; i < 4; i++)
            {
                (OrientationOffsets[i], Amplitudes[i]) = RecoverParameters(initialFootPositions[i].x, initialFootPositions[i].z, initialPhases[i]);
                Debug.Log($"Recovered Parameters for Foot {i}: OrientationOffset: {OrientationOffsets[i]}, Amplitude: {Amplitudes[i]}");
            }

        }

        /// <summary>
        /// Calculates the foot position from joint angles using forward kinematics.
        /// </summary>
        /// <param name="yaw">The yaw angle of the leg.</param>
        /// <param name="hip">The hip angle of the leg.</param>
        /// <param name="knee">The knee angle of the leg.</param>
        /// <returns>The foot position as a Vector3.</returns>
        private Vector3 ForwardKinematics(float yaw, float hip, float knee)
        {
            // Position in the leg plane
            float Xp =
                a +
                c * Mathf.Cos(hip) +
                d * Mathf.Cos(hip + knee);

            float Yp =
                c * Mathf.Sin(hip) +
                d * Mathf.Sin(hip + knee);

            // Distance from yaw axis
            float r = Mathf.Sqrt(Xp * Xp + b * b);

            // relative coordinates
            float x = -r * Mathf.Cos(yaw);
            float y = -Yp;
            float z = r * Mathf.Sin(yaw);

            return new Vector3(x, y, z);
        }

        /// <summary>
        /// Recovers CPG parameters (orientation offset and amplitude) from a foot position.
        /// </summary>
        /// <param name="x">The x-coordinate of the foot position.</param>
        /// <param name="z">The z-coordinate of the foot position.</param>
        /// <param name="phase">The current phase of the CPG oscillator.</param>
        /// <returns>A tuple containing the orientation offset and amplitude.</returns>
        private (float OrientationOffsets, float amplitude) RecoverParameters(float x, float z, float phase)
        {
            Debug.Log("Parameter Recovery called");
            float OrientationOffsets = MathF.Atan2(-z, -x);
            float radius = MathF.Sqrt(x * x + z * z);
            float amplitude = 1.0f + radius / (dStep * MathF.Abs(MathF.Cos(phase)));


            return (OrientationOffsets, amplitude);
        }
    }   
}