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

    [Header("Spine actuator")]
    public Hinge spine;

    [Header("Spine link")]
    public ArticulationBody spine_Link;

    [Header("Spine stiffness")]
    public float spineStiffness = 500f;

    // =========================================================
    // LEG GEOMETRY
    // =========================================================

    [Header("Leg geometry (meters)")]
    public float a = 0.116f;
    public float b = 0.09f;
    public float c = 0.172f;
    public float d = 0.2f;


    // =========================================================
    // CPG PARAMETERS
    // =========================================================

    // Leg Order: FL, FR, RL, RR
    [Header("CPG Parameters")]) 
    public float[] initialPhases = new float[6] { 0f, Mathf.PI, Mathf.PI, 0f , 0f, Mathf.PI};       // Phases for each leg and spine
    public float[] initialAmplitudes = new float[6] { 1f, 1f, 1f, 1f, 1f, 1f};                      // Amplitudes for each leg and spine

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
    private float[] FootRotations;      // Orientation of the foot trajectory, Phi
    private float[] FootRotationShifts; // Change in foot Trajectory orientation, Phi'  controlled via psy



    // =========================================================
    // Functions
    // =========================================================

    public void Initialize()
    {
        // Initialize the CPG parameters and any necessary state variables
        // This may include setting initial frequencies, phases, and amplitudes

        // Attatch Ik Unit if necessarry

        Reset(); // Call Reset to set initial values

    }

    public void Reset()
    {
        // Reset the CPG parameters to their initial values

        Phases = (float[])initialPhases.Clone();                    // Reset phases to initial values
        PhaseShifts = new float[6] { 1f, 1f, 1f, 1f, 1f, 1f };      // Reset phase shifts to default values
        Amplitudes = (float[])initialAmplitudes.Clone();            // Reset amplitudes to initial values
        AmplitudeShifts = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f };  // Reset amplitude shifts to default values
        AmplitudeShifts2 = new float[6] { 0f, 0f, 0f, 0f, 0f, 0f }; // Reset amplitude shifts to default values
        FootRotations = new float[4] { 0f, 0f, 0f, 0f };            // Reset foot rotations to default values
        FootRotationShifts = new float[4] { 0f, 0f, 0f, 0f };       // Reset foot rotation shifts to default values

    }

    public void Update()
    {

        // update CPG
        Phases = Phases + PhaseShifts * TimeStep;                           // Update phases based on phase shifts
        Amplitudes = Amplitudes + AmplitudeShifts * TimeStep;               // Update amplitudes based on amplitude shifts
        AmplitudeShifts = AmplitudeShifts + AmplitudeShifts2 * TimeStep;    // Update amplitude shifts based on second derivative
        FootRotations = FootRotations + FootRotationShifts * TimeStep;      // Update foot rotations based on rotation shifts

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
            FootRotationShifts[i] = continuous[i + 12]; // Update foot rotation shifts
        }
    }


    // =========================================================
    // Inverse Kinematics
    // =========================================================


}
