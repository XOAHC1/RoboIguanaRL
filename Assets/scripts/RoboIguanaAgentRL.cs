using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

namespace RoboIguanaRL
{
    /// <summary>
    /// RoboIguana reinforcement learning agent that controls locomotion through a CPG controller.
    /// </summary>
    public class RoboIguanaAgentRL : Agent
    {
        [Header("Contact Sensors")]
        /// <summary>Contact detectors for the four feet (Front-Left, Front-Right, Rear-Left, Rear-Right).</summary>
        public ContactDetector footFL, footFR, footRL, footRR;

        /// <summary>
        /// Contact detector for the back of the robot, to abort training in unsolvable postions.
        /// </summary> 
        public ContactDetector Back;

        [Header("Articulation Body")]
        /// <summary>The main articulation body representing the robot's physical body.</summary>
        public ArticulationBody Body;
        public Transform BodyPositon;

        /// <summary>Central Pattern Generator controller for managing limb oscillations.</summary>
        private RoboIguanaCPGController CPG;

        /// <summary>Target direction for locomotion.</summary>
        private Vector3 TargetDirection;
        /// <summary>Target velocity in meters per second.</summary>
        private float TargetVelocity;

        private Vector3 StartingPosition;
        private Quaternion StartingOrientation;


        /// <summary>
        /// Initializes the agent by setting up the CPG controller and resetting the target.
        /// </summary>
        public override void Initialize()
        {
            Debug.Log("RoboIguanaAgentRL: Initialize");

            //  Save Starting Position
            BodyPositon.GetPositionAndRotation(out StartingPosition, out StartingOrientation);

            CPG = GetComponent<RoboIguanaCPGController>();
            
            CPG.InitializeCPG();
            ResetTarget();
        }

        /// <summary>
        /// Resets the agent's state, CPG and robot position.
        /// </summary>
        public void Reset()
        {            
            SetReward(0f);
            // Reset foot contact booleans
            footFL.Reset();
            footFR.Reset();
            footRL.Reset();
            footRR.Reset();

            // Reset Back contact
            Back.Reset();

            // Reset CPG
            CPG.Reset();
            
            // Reset Position and Rotation
            BodyPositon.SetPositionAndRotation(StartingPosition, StartingOrientation);
        }

        /// <summary>
        /// Called at the beginning of each episode to reset the agent's state, and target.
        /// </summary>
        public override void OnEpisodeBegin()
        {
            Debug.Log("Starting new Epsode");
            ResetTarget();
            Reset();
        }

        /// <summary>
        /// Collects state observations and adds them to a VectorSensor.
        /// <remarks>
        /// Observed are:
        ///     World State:
        ///         Direction deviation from target 3D
        ///         Velocity deviation from target  1D
        ///         angular velocty                 3D
        ///         Ground contact booleans         4D
        ///     CPG State:
        ///         Phases                          6D
        ///         Amplitudes                      6D
        ///         Orientation Offsets             4D
        /// For a total of 29 input dimensions.
        /// </remarks>
        /// </summary>
        /// <param name="sensor">The vector sensor to add observations to.</param>
        public override void CollectObservations(VectorSensor sensor)
        {
            // position and velocity observations
            sensor.AddObservation(transform.forward - TargetDirection);
            sensor.AddObservation(Body.linearVelocity / TargetVelocity);
            sensor.AddObservation(Body.angularVelocity);

            // Contact Booleans
            sensor.AddObservation(footFR.IsTouchingGround);
            sensor.AddObservation(footFL.IsTouchingGround);
            sensor.AddObservation(footRL.IsTouchingGround);
            sensor.AddObservation(footRR.IsTouchingGround);

            // internal state
            sensor.AddObservation(CPG.GetPhases());
            sensor.AddObservation(CPG.GetAmplitudes());
            sensor.AddObservation(CPG.GetOrientationOffsets());

        }

        /// <summary>
        /// Relays actions received from the policy to control CPG parameters.
        /// <remarks>
        /// Possible actions are: 
        ///     for each limb oscillator:
        ///         change intrinsic frequency  4D
        ///         change amplitude            4D
        ///         change orientation          4D
        ///     for spine and tail:
        ///         change intrinsic frequency  2D
        ///         change amplitude            2D
        /// For a total of 16 action dimensions.
        /// </remarks>
        /// </summary>
        /// <param name="buffers">The action buffers containing the policy decisions.</param>
        public override void OnActionReceived(ActionBuffers buffers)
        {
            // Debug.Log("Actions Received");
            CPG.ApplyActions(buffers);
        }

        /// <summary>
        /// Resets the target direction and velocity. 
        /// <remark> In optimal deployment, this would be called by an independent agent or human.
        /// For training, uses random values; for testing, uses fixed values.
        /// </remark>
        /// </summary>
        public void ResetTarget()
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
                TargetDirection = Vector3.forward;
                TargetVelocity = 3f;
            }
        }

        /// <summary>
        /// Provides heuristic/manual control for testing purposes by keeping all actions at zero.
        /// </summary>
        /// <param name="actionsOut">The action buffers to write heuristic actions to.</param>
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