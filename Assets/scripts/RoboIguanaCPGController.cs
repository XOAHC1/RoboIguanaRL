using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Hinge = VehicleComponents.Actuators.Hinge;

public class RoboIguanaCPGController
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

    [Header("Spine link")]
    public ArticulationBody spine_Link;

    [Header("Tail link")]
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
    public float dStep = 10f;
    public float gC = 15f;
    public float gP = 2f;
    public float h = 50f;

    public float spineRange;

    // =========================================================
    // CPG PARAMETERS
    // =========================================================

    // Leg Order: FL, FR, RL, RR
    [Header("CPG Parameters")]) 
    public float[] initialPhases = new float[6] { 0f, Mathf.PI, Mathf.PI, 0f, 0f, Mathf.PI };       // Phases for each leg and spine
    public float[] initialAmplitudes = new float[6] { 1f, 1f, 1f, 1f, 1f, 1f };                      // Amplitudes for each leg and spine

    public float TimeStep = 0.01f; // Time step for CPG updates (seconds)

    // internal CPG Parameters
    // change in trajectory rotation ($\Phi$)
    // phase speed
    // amplitude increase
    // amplitude 2nd derivative
    //      Spine:
    // amplitude 1st and second derivative
    // phase 1st derivative

    private float[] Phases;             // CPG Phases, Theta
    private float[] PhaseShifts;        // CPG Phase shifts, Theta',                    controlled via omega
    private float[] Amplitudes;         // intrinsic amplitudes, r
    private float[] AmplitudeShifts;    // Shift in amplitude, r'
    private float[] AmplitudeShifts2;   // Shift in amplitude Shifts, r''               controlled via mu
    private float[] OrientationOffset;      // Orientation of the foot trajectory, Phi
    private float[] OrientationOffsetShifts; // Change in foot Trajectory orientation, Phi'  controlled via psy



    // =========================================================
    // Functions
    // =========================================================

    public void Initialize()
    {
        // set CPG parameters to their initial values
        Reset();
    }

    public void Reset()
    {
        // Reset the CPG parameters to their initial values

        Phases = (float[])initialPhases.Clone();                    // Reset phases to initial values
        PhaseShifts = new float[6] { 1f, 1f, 1f, 1f, 1f, 1f };      // Reset phase shifts to default values
        Amplitudes = (float[])initialAmplitudes.Clone();            // Reset amplitudes to initial values
        AmplitudeShifts = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };  // Reset amplitude shifts to default values
        AmplitudeShifts2 = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f }; // Reset amplitude shifts to default values
        OrientationOffset = new float[4] { 0f, 0f, 0f, 0f };            // Reset foot rotations to default values
        OrientationOffsetShifts = new float[4] { 0f, 0f, 0f, 0f };       // Reset foot rotation shifts to default values

    }

    public void Update()
    {
        // update CPG
        Phases = Phases + PhaseShifts * TimeStep;                                       // Update phases based on phase shifts
        Amplitudes = Amplitudes + AmplitudeShifts * TimeStep;                           // Update amplitudes based on amplitude shifts
        AmplitudeShifts = AmplitudeShifts + AmplitudeShifts2 * TimeStep;                // Update amplitude shifts based on second derivative
        OrientationOffset = OrientationOffset + OrientationOffsetShifts * TimeStep;     // Update foot rotations based on rotation shifts

        // update limb positions
        var yaws = new float[3];
        var hips = new float[3];
        var knees = new float[3];

        for (int i = 0; i < 4; i++) {
            (float x, float y, float z) p = GetFootPosition(Phases[i], Amplitudes[i], OrientationOffset[i]);
            (yaws[i], hips[i], knees[i]) = InverseKinematics(p); 
        }
        ApplyAngles(yaws, hips, knees)

        // update spine position
        var float[] spineAngles = new float[2];
        for (int i = 0; i < 2; i++) {
            spineAngles[i] = GetSpineAngles(Phases[i + 4], Amplitudes[i + 4]);
        }
        ApplySpineAngle(spineAngles)

    }

    public void ApplyActions(ActionBuffer actions)
    {
        // Apply the actions received from the RL agent to the CPG parameters
        ActionSegment<float> continuous = actions.ContinuousActions;

        // Assuming the action space is structured as follows:
        //      0-5: Phase shifts for each leg and spine
        //      6-11: Amplitude shifts for each leg and spine
        //      12-15: Foot rotation shifts for each leg

        for (int i = 0; i < 6; i++)
        {
            PhaseShifts[i] = continuous[i];          // Update phase shifts
            AmplitudeShifts[i] = continuous[i + 6];  // Update amplitude shifts
        }

        for (int i = 0; i < 4; i++)
        {
            OrientationOffsetShifts[i] = continuous[i + 12]; // Update foot rotation shifts
        }
    }


    // =========================================================
    // CPG translation
    // =========================================================

    // get limb position from CPG State
    public (float x, float y, float z) GetFootPosition(float phase, float amplitude, float orientationOffset)
    {
        float x = -dStep * (amplitude - 1.0f) * MathF.Cos(phase) * MathF.Sin(orientationOffset);
        float y = -dStep * (amplitude - 1.0f) * MathF.Cos(phase) * MathF.Cos(orientationOffset);
        float z = -h + (MathF.Sin(phase) > 0.0f ? gC : gP) * MathF.Sin(phase);

        return (x, y, z);
    }

    // returns angels for spine and tail from CPG state
    public float GetSpineAngles(float phase, float amplitude)
    {
        return MathF.Sin(phase) * amplitude * spineRange;
    }


    // =========================================================
    // Inverse Kinematics
    // =========================================================

    // Get joint angles from Foot position
    (float yaw, float hip, float knee) InverseKinematics((float x, float y, float z) p)
    {
        float r2 = x * x + z * z;
        float Lmag = Mathf.Sqrt(Mathf.Max(0f, r2 - b * b));

        theta1 = Mathf.Atan2(z, -x);

        float Xp = Lmag - a;
        float Yp = -y;

        float D = (Xp * Xp + Yp * Yp - c * c - d * d) / (2f * c * d);
        D = Mathf.Clamp(D, -1f, 1f);

        theta4 = Mathf.Atan2(Mathf.Sqrt(1 - D * D), D);

        theta3 = Mathf.Atan2(Yp, Xp) -
                 Mathf.Atan2(d * Mathf.Sin(theta4),
                             c + d * Mathf.Cos(theta4));

        return (theta1, theta3, theta4);
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
        spine_Link.SetAngle(angle[0]);
        tail_Link.SetAngle(angles[1]);
    }
}   
