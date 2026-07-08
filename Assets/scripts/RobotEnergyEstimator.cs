using UnityEngine;

/// <summary>
/// Central power estimator for a robot. Manages several <c>PowerServoEstimators</c> for individual robot joints.
/// </summary>
public class RobotEnergyEstimator : MonoBehaviour
{

    /// <summary>
    /// Array of power estimators. Contains estimators for all joints of the robot.
    /// </summary>
    private ServoPowerEstimator[] Estimators;

    /// <summary>
    /// Energy consumption of the robot in the las FixedUpdate step.
    /// </summary>
    public float CurrentEnergy;

    /// <summary>
    /// Total energy consumption of the robot since last reset.
    /// </summary>
    public float CumulatedEnergy;

    /// <summary>
    /// Connects to joint power etimators.
    /// </summary>
    void Start()
    {
        Estimators = GetComponentsInChildren<ServoPowerEstimator>();
        Reset();

        Debug.Log($"Started Energy Estimator for {Estimators.Length} joints");
    }

    /// <summary>
    /// Sets Energy values to zero.
    /// </summary>
    public void Reset()
    {
        CurrentEnergy = 0f;
        CumulatedEnergy = 0f;

        foreach (var est in Estimators)
        {
            est.ResetEnergy();
        }
    }

    /// <summary>
    /// Retrieves current energy consumption and updates cumulated energy.
    /// </summary>
    void FixedUpdate()
    {
        EstimateEnergy();

        CumulatedEnergy += CurrentEnergy;
    }

    /// <summary>
    /// Calculates current energy consumption.
    /// </summary>
    void EstimateEnergy()
    {
        CurrentEnergy = 0f;
        foreach (var est in Estimators)
        {
            CurrentEnergy += est.mechanicalEnergyJ;
        }
    }

}
