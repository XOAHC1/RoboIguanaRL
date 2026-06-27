using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class RoboIguanaAgentRL : Agent
{
    [Header("Contact Sensors")]
    public ContactDetector footFL, footFR, footHL, footHR;

    private Rigidbody rb;
    private RoboIguanaCPGController CPG;
    private DecisionRequester decisionRequester;
    private Vector3 TargetDirection;
    private float TargetVelocity;


    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        CPG = GetComponent<RoboIguanaCPGController>();
        decisionRequester = GetComponent<DecisionRequester>();
        TargetDirection = Vector3.forward;                              // Initialize target direction
        TargetVelocity = 3f;                                            // Initialize target velocity
    }


    public override void OnEpisodeBegin()
    {
        // Reset environment and agent state here

        // Random horizontal direction (unit vector)
        float DirectionRange = 1f; // Range for random direction
        float randX = Random.Range(-DirectionRange, DirectionRange);
        float randZ = Random.Range(-DirectionRange, DirectionRange);
        TargetDirection = new Vector3(randX, 0f, randZ);

        // Random target velocity in a reasonable range (meters per second)
        TargetVelocity = Random.Range(0.1f, 5f);

        CPG.reset();

        Debug.Log($"New Episode: Target Direction = {TargetDirection}, Target Velocity = {TargetVelocity}");
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // position and velocity observations
        //sensor.AddObservation(transform.localPosition);       // 3D
        sensor.AddObservation(transform.forward - TargetDirection);  // 3D      Difference between agent's forward direction and target direction
        sensor.AddObservation(rb.velocity / TargetVelocity);    // 3D
        sensor.AddObservation(rb.angularVelocity);              // 3D

        // Contact Booleans
        sensor.AddObservation(footFR.IsTouchingGround);         // 1D
        sensor.AddObservation(footFL.IsTouchingGround);         // 1D
        sensor.AddObservation(footHL.IsTouchingGround);         // 1D
        sensor.AddObservation(footHR.IsTouchingGround);         // 1D

        // internal state
        sensor.AddObservation(CPG.GetPhases());                 // 6D
        sensor.AddObservation(CPG.GetAmplitudes());             // 6D
        sensor.AddObservation(CPG.GetDirectionalOffsets());     // 4D

        // Target related input
        sensor.AddObservation(TargetDirection);                 // 3D
        sensor.AddObservation(TargetVelocity);                  // 1D

    }

    public override void OnActionsReceived(ActionBuffers buffers)
    {
        // Process actions and apply them to the agent
        // Possible actions are: 
        //      for each limb oscillator:
        //          change intrinsic frequency
        //          change amplitude
        //          change orientation
        //      for spine and tail:
        //          change intrinsic frequency
        //          change amplitude

        CPG.applyActions(buffers);

    }


}
