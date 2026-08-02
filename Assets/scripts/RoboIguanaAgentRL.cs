using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
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

        /// <summary>
        /// Helper object to handle reward weight import and reward logging.
        /// </summary>
        private TrainingManager training;

        /// <summary>Central Pattern Generator controller for managing limb oscillations.</summary>
        private RoboIguanaCPGController CPG;

        /// <summary>
        /// Estimates the energy consumption of the robot.
        /// </summary>
        private RobotEnergyEstimator EnergyEstimator;

        /// <summary>Target direction for locomotion.</summary>
        /// <remarks>Relative to the robot: [yaw, pitch]</remarks>
        private Vector2 TargetAngularVelocity;

        /// <summary>Target velocity in meters per second.</summary>
        /// <remarks>Relative to the robot, x,y</remarks>
        private Vector2 TargetLinearVelocity;

        /// <summary> Type of locomotion requested by higher level controller. </summary>
        /// <remarks>  0: swimming, 1: walking. </remarks>
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

            // Get components
            CPG = GetComponent<RoboIguanaCPGController>();
            CPG.Initialize();
            EnergyEstimator = GetComponent<RobotEnergyEstimator>();
            ComponentABs = GetComponentsInChildren<ArticulationBody>();
            training = new TrainingManager();
            TargetAngularVelocity = Vector2.zero;
            TargetLinearVelocity = Vector2.zero;

            // save starting parameters
            transform.GetPositionAndRotation(out StartingPosition, out StartingOrientation);

            // Apply Config settings
            if (training.Config["Swimming"])
                StartingPosition.y += 1;

            Debug.Log("Agent initialization over");
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
            Debug.Log("Starting new Epsode");
            ResetRobot();
            ResetTarget();
            SetReward(0f);
            training.NewEpisode();
        }

        /// <summary>
        /// Collects state observations and adds them to a VectorSensor.
        /// </summary>
        /// <remarks>
        /// Observed are:
        ///     World State:
        ///         Locomotion type                 [0, 1]
        ///         Target linear velocity          2D
        ///         Linear Velocity                 3D
        ///         Target angular velocity         2D
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
        /// For a total of 52 input dimensions.
        /// </remarks>
        /// <param name="sensor">The vector sensor to add observations to.</param>
        public override void CollectObservations(VectorSensor sensor)
        {
            // Debug.Log("Collecting Observations");

            // position and velocity observations
            sensor.AddObservation(locomotionType);
            sensor.AddObservation(TargetLinearVelocity);
            sensor.AddObservation(transform.InverseTransformDirection(Body.linearVelocity));
            sensor.AddObservation(TargetAngularVelocity);
            sensor.AddObservation(transform.InverseTransformDirection(Body.angularVelocity));

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

            // Tail
            sensor.AddObservation(CPG.GetTailState().Values.ToArray());
        }

        /// <summary>
        /// Relays actions received from the policy to control CPG parameters.
        /// </summary>
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
        /// <param name="buffers">The action buffers containing the policy decisions.</param>
        public override void OnActionReceived(ActionBuffers buffers)
        {
            // Debug.Log("Actions Received");
            CPG.ApplyActions(buffers);
        }

        /// <summary>
        /// Resets the target direction and velocity.
        /// </summary>
        /// <remark> In optimal deployment, this would be called by an independent agent or human.
        /// For training, uses random values; for testing, uses fixed values.
        /// </remark>
        private void ResetTarget()
        {
            // settle locomotion type
            locomotionType = training.Config["Swimming"]? 0: 1;
            locomotionType = training.Config["Transition"]? (locomotionType + 1) % 2: locomotionType;

            // generate target velocities, foreward and upward
            var vel = training.Config["RandomLinearVelocity"] ? 
                // random values
                new Vector2 (
                    Random.Range(0.01f, 0.6f), 
                    (locomotionType == 0) ? 
                        Random.Range(-0.3f, 0.3f) :
                        training.Config["Transition"] ? -0.3f: 0f
                ): 
                // default values:
                new Vector2 (
                    0.3f,
                    0f
                );
            TargetLinearVelocity = vel * (training.Config["Swimming"]? 2f: 1f);
            
            // generate target angular velocities
            TargetAngularVelocity = training.Config["RandomAngularVelocity"] ?
                // random values
                new Vector2 (
                    Random.Range(-0.3f, 0.3f),
                    (locomotionType == 0) ? 
                        Random.Range(-0.2f, 0.2f) :
                        0f
                ): 
                // default values
                new Vector2 (
                    0f,
                    0f
                );

            // Debug.Log($"Target: \n AngVel: {TargetAngularVelocity} \n Velocity: {TargetLinearVelocity} \n locomotion: {locomotionType}");
        }

        /// <summary>
        /// Terminates Episode in unsalvagable situations. Applies reward.
        /// </summary>
        public void FixedUpdate()
        {
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
                training.LinRewards["crash"] = 1/Time.fixedDeltaTime;
                _ = training.GetReward();
                Terminate();
            }
        }

        private void Terminate()
        {
            // Debug.Log($"Terminating Agent. \n Traveled distance: {transform.position - StartingPosition} \n Consumed Energy: {EnergyEstimator.CumulatedEnergy} \n Acheived Reward: {GetCumulativeReward()}");
            EndEpisode();
        }
        
        /// <summary>
        /// Sets raw rewards of <c>training</c> and applies reward to the agent.
        /// </summary>
        private void GiveReward()
        {
            // Any foot touching the ground?
            bool groundContact = footFL.IsTouchingGround || footFR.IsTouchingGround || footRL.IsTouchingGround || footRR.IsTouchingGround;

            // precalculate velocites
            var relVel = transform.InverseTransformDirection(Body.linearVelocity);
            var angVel = transform.InverseTransformDirection(Body.angularVelocity);

            // linear velocity x
            training.ExpRewards["xVel"] = relVel.x - TargetLinearVelocity.x;
            // linear velocity y
            training.ExpRewards["yVel"] = relVel.y - TargetLinearVelocity.y;
            // angular velocity yaw
            training.ExpRewards["yawRate"] = angVel.y - TargetAngularVelocity.x;
            // angular velocity pitch
            training.ExpRewards["pitchRate"] = angVel.z - TargetAngularVelocity.y;
            // linear velocity z
            training.QuadPenalties["zVel"] = relVel.z;
            // angular velocity roll
            training.QuadPenalties["rollRate"] = angVel.x;
            // Work
            training.QuadPenalties["work"] = EnergyEstimator.CurrentEnergy;
            // ground contact
            training.LinRewards["groundContact"] = ((locomotionType == 1) ? 1: -1) * (groundContact ? 1f : -1f);
            // Tail Status
            training.LinRewards["tailStatus"] = ((locomotionType == 1) ? 1: 0) * (CPG.GetTailState()["frequency"] == 0? 0: 1);

            AddReward(training.GetReward());
        }

        /// <summary>
        /// Provides heuristic/manual control for testing purposes by keeping all actions at zero.
        /// </summary>
        /// <param name="actionsOut">The action buffers to write heuristic actions to.</param>
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Provide manual control for testing purposes
            var continuousActionsOut = actionsOut.ContinuousActions;
            var discreteActionsOut = actionsOut.DiscreteActions;
            // Phase shifts
            for (int i = 0; i < 6; i++)                             continuousActionsOut[i] = 0.3f;
            // everything else
            for (int i = 6; i < continuousActionsOut.Length; i++)   continuousActionsOut[i] = 0f;
            
            // continuousActionsOut[continuousActionsOut.Length-1] = 1f;
            discreteActionsOut[0] = 1;
            discreteActionsOut[1] = 1;
            }
    }
}