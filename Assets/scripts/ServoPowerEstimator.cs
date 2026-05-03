using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(ArticulationBody))]
public class ServoPowerEstimator : MonoBehaviour
{
    [Header("Computed values (read only)")]
    public float driveTorqueNm;      // Estimated PD torque
    public float omegaRadPerSec;     // Joint angular velocity (rad/s)
    public float mechanicalPowerW;   // Instantaneous mechanical power (W)

    [Header("Energy")]
    public float mechanicalEnergyJ;  // Accumulated mechanical energy (J)

    public float jointAngleRad;
    public float targetAngleRad;

    public float stiffness;
    public float damping;

    private ArticulationBody joint;

    void Awake()
    {
        joint = GetComponent<ArticulationBody>();
    }

    void FixedUpdate()
    {
        ArticulationDrive drive = joint.xDrive;

        // -------------------------
        // Joint state (radians)
        // -------------------------
        jointAngleRad = joint.jointPosition[0];
        omegaRadPerSec = joint.jointVelocity[0];

        // -------------------------
        // Convert target (deg → rad)
        // -------------------------
        targetAngleRad = drive.target * Mathf.Deg2Rad;
        float targetVelRad = drive.targetVelocity * Mathf.Deg2Rad;

        // -------------------------
        // PD torque calculation
        // τ = Kp*error + Kd*vel_error
        // -------------------------
        float posError = targetAngleRad - jointAngleRad;
        float velError = targetVelRad - omegaRadPerSec;

        stiffness = drive.stiffness;
        damping = drive.damping;

        driveTorqueNm =
            drive.stiffness * posError +
            drive.damping * velError;

        // Apply torque limit
        driveTorqueNm = Mathf.Clamp(
            driveTorqueNm,
            -drive.forceLimit,
            drive.forceLimit);

        // -------------------------
        // Mechanical power
        // -------------------------
        mechanicalPowerW = driveTorqueNm * omegaRadPerSec;

        // Only count positive power (motor consuming energy)
        float consumedPower = Mathf.Max(0f, mechanicalPowerW);

        // Integrate mechanical energy
        mechanicalEnergyJ += consumedPower * Time.fixedDeltaTime;
    }

    // Optional: Reset energy counter
    public void ResetEnergy()
    {
        mechanicalEnergyJ = 0f;
    }
}