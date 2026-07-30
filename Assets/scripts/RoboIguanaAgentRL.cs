using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Collections.Generic;
using System.Linq;

namespace RoboIguanaRL
{
    /// <summary>
    /// RoboIguana reinforcement learning agent that controls locomotion through a CPG controller.
    /// </summary>
    public class RoboIguanaAgentRL : Agent
    {
        /// <summary>Contact detector for the foot.</summary>
        [Header("Contact Sensors")]
        public ContactDetector footFL, footFR, footRL, footRR;

        /// <summary>
        /// Contact detector for the back of the robot, to abort training in unsolvable postions.
        /// </summary> 
        public ContactDetector Back;

        /// <summary>The main articulation body representing the robot's physical body.</summary>
        [Header("Articulation Body")]
        public ArticulationBody Body;

        /// <summary> Weight for reward calculation.</summary>
        [Header("Reward Weights")]
        private readonly float[] RewardWeights = new float[] {
            -4f, // VelocityWeight
            -5f, // DirectionWeight
            -1f, // RollWeight
            10f, // GroundContactWeight
            -1f, // EnergyConsumptionWeight
            -2f // TailWhileWalkingWeight
        }; 

        private float[] Rewards = new float[6];

        public Dictionary<string, List<float>> RewardTracker = new Dictionary<string, List<float>>();

        /// <summary>Central Pattern Generator controller for managing limb oscillations.</summary>
        private RoboIguanaCPGController CPG;

        /// <summary>
        /// Estimates the energy consumption of the robot.
        /// </summary>
        private RobotEnergyEstimator EnergyEstimator;

        /// <summary>Target direction for locomotion.</summary>
        private Vector3 TargetDirection;

        /// <summary>Target velocity in meters per second.</summary>
        private Vector3 TargetVelocity;

        /// <summary>
        /// Type of locomotion requested by higher level controller. 
        /// <remarks> 1 = walking, -1 = swimming. </remarks>
        /// </summary>
        private int locomotionType;

        /// <summary>
        /// Initial positon of the robot.
        /// </summary>
        private Vector3 StartingPosition;

        /// <summary>
        /// Initial orientation of the robot.
        /// </summary>
        private Quaternion StartingOrientation;

        /// <summary>
        /// Contains <c>ArticulationBody</c> elements of all components of the robot.
        /// </summary>
        private ArticulationBody[] ComponentABs;

        /// <summary>
        /// Initializes the agent by setting up the CPG controller and resetting the target.
        /// </summary>
        public override void Initialize()
        {
            Debug.Log("RoboIguanaAgentRL: Initialize");

            ResetRewardTracker();
            CPG = GetComponent<RoboIguanaCPGController>();
            EnergyEstimator = GetComponent<RobotEnergyEstimator>();
            ComponentABs = GetComponentsInChildren<ArticulationBody>();
            
            transform.GetPositionAndRotation(out StartingPosition, out StartingOrientation);

            CPG.Initialize();
            Debug.Log("Agent initialization over");
        }

        private void ResetRewardTracker()
        {
           RewardTracker["Velocity"] = new List<float>();
           RewardTracker["Direction"] = new List<float>();
           RewardTracker["Roll"] = new List<float>();
           RewardTracker["Ground contact"] = new List<float>();
           RewardTracker["Energy"] = new List<float>();
           RewardTracker["Tail while walking"] = new List<float>();
        }
        /// <summary>
        /// Resets the Robots Positon, CPG and Sensors.
        /// </summary>
        public void ResetRobot()
        {
            // Reset Robot Position
            Body.TeleportRoot(StartingPosition, StartingOrientation);
            foreach (ArticulationBody ab in ComponentABs)
            {
                ab.linearVelocity = Vector3.zero;
                ab.angularVelocity = Vector3.zero;
            }

            CPG.Reset();

            // Reset foot contact sensors
            footFL.Reset();
            footFR.Reset();
            footRL.Reset();
            footRR.Reset();
            Back.Reset();
        }

        /// <summary>
        /// Called at the beginning of each episode to reset the agent's state, and target.
        /// </summary>
        public override void OnEpisodeBegin()
        {
            Debug.Log($"Last Episode Reward composition: \n    Velocity: {RewardTracker["Velocity"].Sum()} \n    Direction: {RewardTracker["Direction"].Sum()} \n    Roll: {RewardTracker["Roll"].Sum()} \n    Ground contact: {RewardTracker["Ground contact"].Sum()} \n    Energy: {RewardTracker["Energy"].Sum()} \n    Tail while walking: {RewardTracker["Tail while walking"].Sum()}");
            ResetRewardTracker();
            Debug.Log("Starting new Epsode");
            ResetRobot();
            SetReward(0f);
            ResetTarget();


        }


        /// <summary>
        /// Collects state observations and adds them to a VectorSensor.
        /// <remarks>
        /// Observed are:
        ///     World State:
        ///         Direction deviation from target 3D
        ///         Velocity deviation from target  3D
        ///         angular velocty                 3D
        ///         Ground contact booleans         4D
        ///     CPG State:
        ///         Phases                          6D
        ///         Phase shifts                    6D
        ///         Amplitudes                      6D
        ///         Ampltude shifts                 6D
        ///         Orientation offsets             4D
        ///         Orientation offset shifts       4D
        ///     Others:
        ///         Buoyancy                        2D
        ///         Tail State                      3D
        /// For a total of 51 input dimensions.
        /// </remarks>
        /// </summary>
        /// <param name="sensor">The vector sensor to add observations to.</param>
        public override void CollectObservations(VectorSensor sensor)
        {

            // Debug.Log("Collecting Observations");
            // position and velocity observations
            sensor.AddObservation(locomotionType);
            sensor.AddObservation(transform.InverseTransformDirection(TargetDirection));
            sensor.AddObservation(transform.InverseTransformDirection(TargetVelocity - Body.linearVelocity));
            sensor.AddObservation(Body.angularVelocity);

            // Contact Booleans
            sensor.AddObservation(footFR.IsTouchingGround);
            sensor.AddObservation(footFL.IsTouchingGround);
            sensor.AddObservation(footRL.IsTouchingGround);
            sensor.AddObservation(footRR.IsTouchingGround);

            // internal state
            sensor.AddObservation(CPG.GetPhases());
            sensor.AddObservation(CPG.GetPhaseShifts());
            sensor.AddObservation(CPG.GetAmplitudes());
            sensor.AddObservation(CPG.GetAmplitudeShifts());
            sensor.AddObservation(CPG.GetOrientationOffsets());
            sensor.AddObservation(CPG.GetOrientationOffsetShifts());

            // Buoyancy
            sensor.AddObservation(CPG.GetBuoyancy());
            sensor.AddObservation(CPG.GetBuoyancyShift());

            sensor.AddObservation(CPG.GetTailState().Values.ToArray());
        }

        /// <summary>
        /// Relays actions received from the policy to control CPG parameters.
        /// <remarks>
        /// Possible actions are: 
        ///   continuous:
        ///     for each limb oscillator:
        ///         change intrinsic frequency  4D
        ///         change amplitude            4D
        ///         change orientation          4D
        ///     for the spine:
        ///         change intrinsic frequency  2D
        ///         change amplitude            2D
        ///     buoyancy:
        ///         change in buoyancy          1D
        ///   discrete:
        ///     Tail:                           Will be translated to [-1, 0, 1] later on.
        ///         frequency                   [0, 1, 2]
        ///         yaw amplitude               [0, 1, 2]
        ///         
        /// For a total of 19 action dimensions.
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
        private void ResetTarget()
        {
            // turn on for training, turn off for testing
            bool randomMode = false;
            if (randomMode) { 
                // Random horizontal direction (unit vector)
                Vector2 direction = Random.onUnitCircle;
                TargetDirection = new Vector3(direction.x, 0f, direction.y);

                // Random target velocity in a reasonable range (meters per second)
                TargetVelocity = Random.Range(0.1f, 3f) * TargetDirection;
            } else {
                // Fixed target direction and velocity for testing
                TargetDirection = Vector3.forward;
                TargetVelocity = 3f * TargetDirection;
            }
            locomotionType = 1;
        }

        /// <summary>
        /// Terminates Episode in unsalvagable situations. Applies reward.
        /// </summary>
        public void FixedUpdate()
        {
            // Debug.Log("Fixed Update Agent");
            TerminateIfNecessary();
            GiveReward();
        }

        /// <summary>
        /// Checks termination condtions and terminates the episode if appropriate.
        /// </summary>
        private void TerminateIfNecessary()
        {
            if (Back.IsTouchingGround)
            {
                Debug.Log("Back is on the ground!");
                Terminate();
            }
        }

        private void Terminate()
        {
            Debug.Log($"Terminating Agent. \n Traveled distance: {transform.position - StartingPosition} \n Consumed Energy: {EnergyEstimator.CumulatedEnergy} \n Acheived Reward: {GetCumulativeReward()}");
            EndEpisode();
        }
        
        /// <summary>
        /// Computes the reward from multiple components and applies it to the agent.
        /// </summary>
        /// <remarks>
        /// <list type="table">
        ///   <listheader>
        ///     <term>Parameter</term>
        ///     <description>Description</description>
        ///   </listheader>
        ///   <item>
        ///     <term>VelocityError</term>
        ///     <description>Penalizes deviation from target speed.</description>
        ///   </item>
        ///   <item>
        ///     <term>DirectionError</term>
        ///     <description>Penalizes deviation from the target heading.</description>
        ///   </item>
        ///   <item>
        ///     <term>PitchPenalty</term>
        ///     <description>Penalizes excessive pitch angular velocity.</description>
        ///   </item>
        ///   <item>
        ///     <term>RollPenalty</term>
        ///     <description>Penalizes excessive roll angular velocity.</description>
        ///   </item>
        ///   <item>
        ///     <term>GroundContact</term>
        ///     <description>Rewards contact of one or more feet with ground.</description>
        ///   </item>
        ///   <item>
        ///     <term>EnergyConsumption</term>
        ///     <description>Penalizes energy usage.</description>
        ///   </item>
        /// </list>
        /// </remarks>
        private void GiveReward()
        {
            // linear velocity
            var VelError = transform.InverseTransformDirection(TargetVelocity - Body.linearVelocity).sqrMagnitude;

            // Direction
            var DirectionDifference = transform.InverseTransformDirection(TargetDirection - transform.forward).sqrMagnitude;

            // undesired roll
            float RollPenalty = Mathf.Abs(Body.angularVelocity[2]) * Mathf.Abs(Body.angularVelocity[2]);
            
            // Any foot touching the ground?
            bool groundContact = footFL.IsTouchingGround || footFR.IsTouchingGround || footRL.IsTouchingGround || footRR.IsTouchingGround;
            float GroundContact = locomotionType * (groundContact ? 1f : -1f);
            
            float TailMovementWhenWalking = (locomotionType == 1) ? CPG.GetTailState()["frequency"] : 0;

            // EnergyConsumption
            float EnergyConsumption = EnergyEstimator.CurrentEnergy;

            Rewards = new float[] {
                VelError,
                DirectionDifference,
                RollPenalty,
                GroundContact,
                TailMovementWhenWalking,
                EnergyConsumption,
            };

            // Apply Rewards
            float stepReward = 0f;

            for (int i = 0; i < 6; i++)
            {
                var partialReward = Rewards[i] * RewardWeights[i];
                stepReward += partialReward;
                RewardTracker[RewardTracker.Keys.ToArray()[i]].Add(partialReward);
            }

            AddReward(stepReward);
 
        }

        /// <summary>
        /// Provides heuristic/manual control for testing purposes by keeping all actions at zero.
        /// </summary>
        /// <param name="actionsOut">The action buffers to write heuristic actions to.</param>
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Provide manual control for testing purposes
            var continuousActionsOut = actionsOut.ContinuousActions;
            // Phase shifts
            for (int i = 0; i < 6; i++)                             continuousActionsOut[i] = 0.2f;
            // everything else
            for (int i = 6; i < continuousActionsOut.Length; i++)   continuousActionsOut[i] = 0f;
            // continuousActionsOut[continuousActionsOut.Length - 2] = 1f;
            // continuousActionsOut[continuousActionsOut.Length - 1] = 1f;
            }
    }
}