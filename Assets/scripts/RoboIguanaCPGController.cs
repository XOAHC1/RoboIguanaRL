using System;
using System.Collections.Generic;
using System.IO;
using Hinge = VehicleComponents.Actuators.Hinge;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class RoboIguanaCPGController: MonoBehaviour
{
    // =========================================================
    // ACTUATORS
    // =========================================================

    [Header("Hip yaw actuators")]
    public Hinge hyFL, hyFR, hyRL, hyRR;

    [Header("Hip pitch actuators")]
    public Hinge hpFL, hpFR, hpRL, hpRR;

    [Header("Knee actuators")]
    public Hinge kFL, kFR, kRL, kRR;

    [Header("Spine actuators")]
    public Hinge spine;
    public Hinge tail;



    // =========================================================
    // PHYSICAL JOINTS
    // =========================================================

    [Header("Hip yaw links")]
    public ArticulationBody hyFL_Link;
    public ArticulationBody hyFR_Link;
    public ArticulationBody hyRL_Link;
    public ArticulationBody hyRR_Link;

    [Header("Hip pitch links")]
    public ArticulationBody hpFL_Link;
    public ArticulationBody hpFR_Link;
    public ArticulationBody hpRL_Link;
    public ArticulationBody hpRR_Link;

    [Header("Knee links")]
    public ArticulationBody kFL_Link;
    public ArticulationBody kFR_Link;
    public ArticulationBody kRL_Link;
    public ArticulationBody kRR_Link;

    // =========================================================
    // SPINE
    // =========================================================

    [Header("Spine links")]
    public ArticulationBody spine_Link;
    public ArticulationBody tail_Link;

    // =========================================================
    // LEG GEOMETRY
    // =========================================================

    [Header("Leg geometry (meters)")]
    public float a = 0.116f;
    public float b = 0.09f;
    public float c = 0.172f;
    public float d = 0.2f;

    // =========================================================
    // Trajectory parameters
    // =========================================================

    [Header("Trajectory Parameters")]
    public float dStep = 0.15f;         // Step length of the foot trajectory
    public float gC = 0.04f;             // ground clearance
    public float gP = 0.03f;            // ground penetration
    public float h = 0.18f;             // height of the robot

    public float spineRange = 20f;
    public float tailRange = 20f;

    [Header("Convergence Parameters")]
    public float convergence = 0.1f;    // Convergence rate for amplitude shifts. in literature as a
    public float TimeStep = 0.01f;      // Time step for CPG updates (seconds)

    // Initial foot positions for each leg (FL, FR, RL, RR)
    private Vector3[] initialFootPositions = new Vector3[4] {
        new Vector3(0.075f, -0.18f, 0.25f),   // FL
        new Vector3(-0.075f, -0.18f, -0.25f), // FR
        new Vector3(-0.075f, -0.18f, 0.25f),  // RL
        new Vector3(0.075f, -0.18f, -0.25f)   // RR
    };

    public float TimeStep = 0.01f; // Time step for CPG updates (seconds)

    // =========================================================
    // CPG PARAMETERS       Leg Order: FL, FR, RL, RR
    // =========================================================

    // initial Parameters to reset to. Current Values are set to match the initial foot positions from the original project
    private float[] initialPhases = new float[6] { 0f, Mathf.PI, Mathf.PI, 0f, Mathf.PI * 3 / 2, Mathf.PI * 3 / 2};     // Phases for each leg and spine
    private float[] initialPhaseShifts = new float[6] { 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };                           // Phase shifts for each leg and spine
    private float[] initialAmplitudes = new float[6] { 2.740051f, 2.740051f, 2.740051f, 2.740051f, 0.1f, 0.1f };            // Amplitudes for each leg and spine
    private float[] initialAmplitudeShifts = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };                                   // Amplitude shifts for each leg and spine
    private float[] initialAmplitudeShifts2 = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };                                  // Amplitude shifts for each leg and spine
    private float[] initialOrientationOffset = new float[4] { -1.862253f, -1.862253f, 1.862253f, 1.862253f };           // Orientation offsets for each leg
    private float[] initialOrientationOffsetShifts = new float[4] { 0f, 0f, 0f, 0f };                                   // Orientation offset shifts for each leg

    // CPG Parameters throughout
    private float[] Phases;                     // CPG Phases, Theta
    private float[] PhaseShifts;                // CPG Phase shifts, Theta',                    controlled via omega
    private float[] Amplitudes;                 // intrinsic amplitudes, r
    private float[] AmplitudeShifts;            // Shift in amplitude, r'
    private float[] AmplitudeShifts2;           // Shift in amplitude Shifts, r''               controlled via mu
    private float[] OrientationOffset;          // Orientation of the foot trajectory, Phi
    private float[] OrientationOffsetShifts;    // Change in foot Trajectory orientation, Phi'  controlled via psy

    // =========================================================
    // Communication with RL Agent
    // =========================================================

    public float[] GetPhases()
    {
        return (float[])Phases.Clone();
    }

    public float[] GetAmplitudes()
    {
        return (float[])Amplitudes.Clone();
    }

    public float[] GetDirectionalOffsets()
    {
        return (float[])OrientationOffset.Clone();
    }


    // =========================================================
    // Internal function
    // =========================================================

    public void InitializeCPG()
    {
        // set CPG parameters to their initial values
        Reset();

        // Initialize all joints to their starting positions
        InitializeAllJoints();
    }

    // Connect hinges and articulation bodies, set initial pose
    private void InitializeAllJoints()
    {
        // Reset the CPG parameters to their initial values

        // group joints for easier access
        ArticulationBody[] hipYawLinks = new ArticulationBody[4] { hyFL_Link, hyFR_Link, hyRL_Link, hyRR_Link };
        Hinge[] hipYawHinges = new Hinge[4] { hyFL, hyFR, hyRL, hyRR };
        ArticulationBody[] hipPitchLinks = new ArticulationBody[4] { hpFL_Link, hpFR_Link, hpRL_Link, hpRR_Link };
        Hinge[] hipPitchHinges = new Hinge[4] { hpFL, hpFR, hpRL, hpRR };
        ArticulationBody[] kneeLinks = new ArticulationBody[4] { kFL_Link, kFR_Link, kRL_Link, kRR_Link };
        Hinge[] kneeHinges = new Hinge[4] { kFL, kFR, kRL, kRR };

        // Initialize joint positions based on CPG parameters
        for (int i = 0; i<4; i++)
        {
            (float x, float y, float z) p = GetFootPosition(initialPhases[i], initialAmplitudes[i], initialOrientationOffset[i]);
            //Vector3 p = initialFootPositions[i];
            Debug.Log($"Initial Foot position for Foot {i}: {p}");
            (float yaw, float hip, float knee) = InverseKinematics(p);

            InitialiseJoint(hipYawLinks[i], hipYawHinges[i], yaw);
            InitialiseJoint(hipPitchLinks[i], hipPitchHinges[i], hip);
            InitialiseJoint(kneeLinks[i], kneeHinges[i], knee);

            //Debug.Log($"Initialized Leg {i}: Foot Position: ({p.x}, {p.y}, {p.z}), Yaw: {yaw}, Hip: {hip}, Knee: {knee}");
    }

        // Spine and tail
        InitialiseJoint(spine_Link, spine, initialPhases[4] * initialAmplitudes[4]);
        InitialiseJoint(tail_Link, tail, initialPhases[5] * initialAmplitudes[5]);

        }

    // Set joint to initial state (position + drive target) and link hinges and articulation bodies
    void InitialiseJoint(ArticulationBody ab, Hinge h, float angleRad)
        {
        if (ab == null) return;

        ab.jointPosition = new ArticulationReducedSpace(angleRad);
        var drive = ab.xDrive;
        drive.target = angleRad;
        ab.xDrive = drive;

        if (h != null)
            h.SetAngle(angleRad);
        }

    public void Reset()
    {
        // Reset the CPG parameters to their initial values

        Phases = (float[])initialPhases.Clone();                                        // Reset phases to initial values
        PhaseShifts = (float[])initialPhaseShifts.Clone();                              // Reset phase shifts to default values
        Amplitudes = (float[])initialAmplitudes.Clone();                                // Reset amplitudes to initial values
        AmplitudeShifts = (float[])initialAmplitudeShifts.Clone();                      // Reset amplitude shifts to default values
        AmplitudeShifts2 = (float[])initialAmplitudeShifts2.Clone();                    // Reset amplitude shifts to default values
        OrientationOffset = (float[])initialOrientationOffset.Clone();                  // Reset foot rotations to default values
        OrientationOffsetShifts = (float[])initialOrientationOffsetShifts.Clone();      // Reset foot rotation shifts to default values

        //UpdatePose(); // Update the robot's pose based on the reset CPG parameters

    }

    public void UpdatePose()
    {
        // update limb positions
        var yaws = new float[4];
        var hips = new float[4];
        var knees = new float[4];

        // update limb positions
        for (int i = 0; i < 4; i++) {
            (float x, float y, float z) p = GetFootPosition(Phases[i], Amplitudes[i], OrientationOffset[i]);
            (yaws[i], hips[i], knees[i]) = InverseKinematics(p);
        }
        ApplyAngles(yaws, hips, knees);

        // update spine position
        float[] spineAngles = new float[2];
        for (int i = 0; i < 2; i++) {
            spineAngles[i] = GetSpineAngles(Phases[i + 4], Amplitudes[i + 4]);
        }
        ApplySpineAngle(spineAngles);
    }

    public void ApplyActions(ActionBuffers actions)
    {
        // Apply the actions received from the RL agent to the CPG parameters
        ActionSegment<float> continuous = actions.ContinuousActions;

        // Assuming the action space is structured as follows:
        //      0-5: Phase shifts for each leg and spine
        //      6-11: Amplitude shifts for each leg and spine
        //      12-15: trajectory rotation shifts for each leg

        // update phase and amplitude for all joints
        for (int i = 0; i < 6; i++)
        {
            // adapt phase shifts
            PhaseShifts[i] = continuous[i];

            // adapt second derivative of amplitude 
            AmplitudeShifts2[i] =  convergence * ((convergence / 4) * (continuous[i + 6] - Amplitudes[i]) - AmplitudeShifts[i]);
        }

        // update trajectory rotation shifts for all legs
        for (int i = 0; i < 4; i++)
        {
            OrientationOffsetShifts[i] = continuous[i + 12]; // Update trajectory rotation shifts
        }
    }


    // =========================================================
    // CPG translation
    // =========================================================

    // get limb position from CPG State
    public (float x, float y, float z) GetFootPosition(float phase, float amplitude, float orientationOffset)
    {
        float x = -dStep * (amplitude - 1.0f) * MathF.Cos(phase) * MathF.Cos(orientationOffset);
        float y  = -h + (MathF.Sin(phase) > 0.0f ? gC : gP) * MathF.Sin(phase);
        float z = -dStep * (amplitude - 1.0f) * MathF.Cos(phase) * MathF.Sin(orientationOffset);

        return (x, y, z);
    }

    // returns angels for spine and tail from CPG state
    public float GetSpineAngles(float phase, float amplitude)
    {
        return MathF.Sin(phase) * amplitude * spineRange;
    }


    // =========================================================
    // Inverse Kinematics (Copied from original Project)
    // =========================================================

    (float yaw, float hip, float knee) InverseKinematics(Vector3 p)
    {
        return InverseKinematics((p.x, p.y, p.z));
    }

    // Get joint angles from Foot position
    (float yaw, float hip, float knee) InverseKinematics((float x, float y, float z) p)
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

    // set limbs to new pose
    void ApplyAngles(
            float[] yaw, float[] hip, float[] knee)
    {
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

    // set Spine and tail to new pose
    void ApplySpineAngle(float[] angles)
    {
        spine.SetAngle(angles[0]);
        tail.SetAngle(angles[1]);
    }
}   
