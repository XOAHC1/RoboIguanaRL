using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Force;
using Unity.AppUI.UI;

namespace RoboIguanaRL
{
    public class RoboIguanaAgentRL : Agent
    {
        [Header("Contact Sensors")]
        public ContactDetector footFL, footFR, footRL, footRR;

        private Rigidbody rb;
        private RoboIguanaCPGController CPG;
        private DecisionRequester decisionRequester;

        // Goal Parameters
        private Vector3 TargetDirection;
        private float TargetVelocity;


        public override void Initialize()
        {
            Debug.Log("RoboIguanaAgentRL: Initialize");
            rb = GetComponent<Rigidbody>();
            CPG = GetComponent<RoboIguanaCPGController>();
            decisionRequester = GetComponent<DecisionRequester>();

            CPG.InitializeCPG();
            ResetTarget();

        }

        public override void OnEpisodeBegin()
        {
            ResetTarget();
            CPG.Reset();

            // Reset contact detectors.
            footFL.Reset();
            footFR.Reset();
            footRL.Reset();
            footRR.Reset();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // position and velocity observations
            //sensor.AddObservation(transform.localPosition);       // 3D
            sensor.AddObservation(transform.forward - TargetDirection);  // 3D      Difference between agent's forward direction and target direction
            sensor.AddObservation(rb.linearVelocity / TargetVelocity);    // 3D
            sensor.AddObservation(rb.angularVelocity);              // 3D

            // Contact Booleans
            sensor.AddObservation(footFR.IsTouchingGround);         // 1D
            sensor.AddObservation(footFL.IsTouchingGround);         // 1D
            sensor.AddObservation(footRL.IsTouchingGround);         // 1D
            sensor.AddObservation(footRR.IsTouchingGround);         // 1D

            // internal state
            sensor.AddObservation(CPG.GetPhases());                 // 6D
            sensor.AddObservation(CPG.GetAmplitudes());             // 6D
            sensor.AddObservation(CPG.GetDirectionalOffsets());     // 4D

            // Target related input
            sensor.AddObservation(TargetDirection);                 // 3D
            sensor.AddObservation(TargetVelocity);                  // 1D

        }

        public override void OnActionReceived(ActionBuffers buffers)
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

            CPG.ApplyActions(buffers);

        }

        // in optimal deployment would be called by independent agent or human
        public void ResetTarget()
        // Randomly set a new target direction and velocity
        {
            // turn on for training, turn off for testing
            bool randomMode = false;
            if (randomMode) { 
                // Random horizontal direction (unit vector)
                Vector2 direction = Random.insideUnitCircle;
                TargetDirection = new Vector3(direction.x, 0f, direction.y);

                // Random target velocity in a reasonable range (meters per second)
                TargetVelocity = Random.Range(0.1f, 5f);
            } else {
                // Fixed target direction and velocity for testing
                TargetDirection = Vector3.forward;          // Initialize target direction
                TargetVelocity = 3f;                        // Initialize target velocity
            }
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Provide manual control for testing purposes
            var continuousActionsOut = actionsOut.ContinuousActions;
            for (int i = 0; i < continuousActionsOut.Length; i++)
            {
                // Keep parameters constant for heuristic.
                continuousActionsOut[i] = 0f;
            }
        }

    }
}