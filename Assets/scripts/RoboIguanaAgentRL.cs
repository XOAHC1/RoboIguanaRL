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
        /// <summary>
        /// Contact detector for the foot.
        /// </summary>
        [Header("Contact Sensors")]
        public ContactDetector footFL, footFR, footRL, footRR;

        /// <summary>
        /// Contact detector for the back of the robot, to abort training in unsolvable postions.
        /// </summary> 
        public ContactDetector Back;

        /// <summary>
        /// The main articulation body representing the robot's physical body.
        /// </summary>
        [Header("Articulation Body")]
        public ArticulationBody Body;

        /// <summary>
        /// Helper object to handle reward weight import and reward logging.
        /// </summary>
        private TrainingManager training;

        /// <summary>
        /// Central Pattern Generator controller for managing limb oscillations.
        /// </summary>
        private RoboIguanaCPGController CPG;

        /// <summary>
        /// Estimates the energy consumption of the robot.
        /// </summary>
        private RobotEnergyEstimator EnergyEstimator;

        /// <summary>
        /// Requests decsions from the Agent.
        /// </summary>
        private DecisionRequester decisionRequester;

        /// <summary>
        /// Target direction for locomotion.
        /// </summary> <remarks>
        /// Relative to the robot: [yaw, pitch]
        /// </remarks>
        private Vector2 TargetAngularVelocity = Vector2.zero;

        /// <summary>
        /// Target velocity in meters per second.
        /// </summary> <remarks>
        /// Relative to the robot, x,y
        /// </remarks>
        private Vector2 TargetLinearVelocity = Vector2.zero;

        /// <summary> 
        /// Type of locomotion requested by higher level controller.
        /// </summary> <remarks>  0: swimming, 1: walking. 
        /// </remarks>
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
        /// Number of physics steps to wait at the begin of an episode, to let the robot settle.
        /// </summary>
        private int waiting;
        private readonly int waitSteps = 150;

        /// <summary>
        /// Number of agent decisions until new target inputs are generated.
        /// </summary>
        private int nextTargetSteps, nextTargetFreq = 120;

        /// <summary>
        /// Number of target resets until locomotion mode is changed when landing.
        /// </summary>
        private int 
            nextLocomotionmode = 2, 
            locomotionModeChange = 2;

        /// <summary>
        /// Initializes the agent by setting up the CPG controller and resetting the target.
        /// </summary>
        public override void Initialize()
        {
            Debug.Log("RoboIguanaAgentRL: Initialize");
            decisionRequester = GetComponent<DecisionRequester>();

            // Get components
            CPG = GetComponent<RoboIguanaCPGController>();
            CPG.Initialize();

            EnergyEstimator = GetComponent<RobotEnergyEstimator>();
            ComponentABs = GetComponentsInChildren<ArticulationBody>();

            training = new TrainingManager();
            CPG.SimpleWalking = training.Config["SimpleWalking"];
            CPG.SimpleSwimming = training.Config["SimpleSwimming"];

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
            // start waiting
            waiting = waitSteps;
            decisionRequester.DecisionPeriod = 99999;

            SetReward(0f);
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
            if (training.Config["Analysis"])
                Debug.Log($"Linear velocity: {transform.InverseTransformDirection(Body.linearVelocity)} \n Angular velocity: {transform.InverseTransformDirection(Body.angularVelocity)}");
            
            if (nextTargetSteps < 2) ResetTarget();
            else nextTargetSteps --;

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
        ///     Tail:                          
        ///         yaw amplitude               [0, 1, 2]   Will be translated to [-1, 0, 1] later on.
        ///         frequency                   [0, 1]      Tail off/on
        ///         
        /// For a total of 19 action dimensions.
        /// </remarks>
        /// <param name="buffers">The action buffers containing the policy decisions.</param>
        public override void OnActionReceived(ActionBuffers buffers)
        {
            if (training.Config["Analysis"])
                Debug.Log($"Agent Actons: Continuous=[{string.Join(", ", buffers.ContinuousActions.ToArray())}], Discrete=[{string.Join(", ", buffers.DiscreteActions.ToArray())}]");
            
            var cont = buffers.ContinuousActions;
            var disc = buffers.DiscreteActions;

            // block and punish undesirable actions in training mode
            if (training.Config["Swimming"] & training.Config["SimpleSwimming"]) {
                for (int i = 0; i < 4; i++) {
                    // apply penalty
                    training.LinRewards["SimpleTrainingPenalties"] = 
                        (cont[i] + 1)/2     +                           // limb frequencies
                        Mathf.Abs(cont[i+12]);                           // limb sideqays phases 
                    // change vaues to neutral behavour
                    cont[i] = -1;
                    cont[i+12] = 0;
                }
            }

            else if (training.Config["Landing"])
            {
                // punish
                training.LinRewards["SimpleTrainingPenalties"] = 
                    cont[16] > 0? cont[16]: 0;
                // block
                cont[16] = Mathf.Clamp(cont[16], -1, 0.5f);
            }

            else if (training.Config["SimpleWalking"])
            {
                training.LinRewards["simpleTrainingPenalties"] = 
                    (buffers.ContinuousActions[4] + 1) /2        +   // spine pitch phase progression
                    buffers.ContinuousActions[16]                +   // buoyancy increase
                    ((buffers.DiscreteActions[0] != 1)? 1f: 0f)  +   // Tail amp
                    ((buffers.DiscreteActions[1] != 1)? 1f: 0f)  ;   // tail freq
                // set values to neutral
                cont[4] = -1;
                cont[16] = -1;
                disc[0] = 0;    disc[1] = 0;
            }

    
            // Debug.Log($"Agent Actons after processing: Continuous=[{string.Join(", ", buffers.ContinuousActions.ToArray())}], Discrete=[{string.Join(", ", buffers.DiscreteActions.ToArray())}]");
            // relay actions to CPG
            CPG.ApplyActions(buffers);
        }

        /// <summary>
        /// Selects new locomotion targets.
        /// </summary> <remark> 
        /// If random target values are disabled, default values will be selected.
        /// </remark>
        private void ResetTarget()
        {
            nextTargetSteps = nextTargetFreq;

            if(training.Config["Transition"] & (training.Config["Swimming"] | training.Config["Landing"]))
            {
                if (nextLocomotionmode < 1)
                {
                    if (training.Config["Swimming"])
                    {
                        training.Config["Swimming"] = false;
                        training.Config["Landing"] = true;
                    }
                    else if (training.Config["Landing"])
                    {
                        training.Config["Landing"] = false;
                    }
                }
                else nextLocomotionmode--;
            }

            // settle locomotion type
            locomotionType = training.Config["Swimming"]? 0: 1;

            // generate target velocities, foreward and upward
            var vel = new Vector2(
                training.Config["RandomXVelocity"] ? Random.Range(0.0f, 0.6f): 0.4f,
                training.Config["RandomYVelocity"] ? Random.Range(-0.4f, 0.4f): 0f
            );
            if (training.Config["Landing"]) vel.y = -0.2f;
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
            
            if (training.Config["Analysis"]) Debug.Log($"New Target: \n LinVel: {TargetLinearVelocity} \n AngVel: {TargetAngularVelocity}");
        }

        /// <summary>
        /// Terminates Episode in unsalvagable situations. Applies reward.
        /// </summary>
        public void FixedUpdate()
        {
            // wait after reset
            if (waiting > 0)
            {
                waiting--;
                // Debug.Log("waiting");
                if (waiting < Time.fixedDeltaTime) decisionRequester.DecisionPeriod = 10;
                return;
            }

            // update Robot
            CPG.Step();
            EnergyEstimator.Step();

            // evaluate step
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

        /// <summary>
        /// Terminates the agent end ends the episode early.
        /// </summary>
        private void Terminate()
        {
            // Debug.Log($"Terminating Agent. \n Traveled distance: {transform.position - StartingPosition} \n Consumed Energy: {EnergyEstimator.CumulatedEnergy} \n Acheived Reward: {GetCumulativeReward()}");
            EndEpisode();
        }
        
        /// <summary>
        /// Gives reward measures to <c>training</c> and applies reward to the agent.
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
            // linear velocity z
            training.QuadPenalties["zVel"] = relVel.z;
            // angular velocity roll
            training.QuadPenalties["rollRate"] = angVel.x;
            // angular velocity yaw
            training.ExpRewards["yawRate"] = angVel.y - TargetAngularVelocity.x;
            // angular velocity pitch
            training.ExpRewards["pitchRate"] = angVel.z - TargetAngularVelocity.y;
            // Work
            training.QuadPenalties["work"] = EnergyEstimator.CurrentEnergy;
            // ground contact
            training.LinRewards["groundContact"] = ((locomotionType == 1) ? 1: -1) * (groundContact ? 1f : -1f);
            // Tail Status
            training.LinRewards["tailStatus"] = ((locomotionType == 1) ? 1: 0) * (CPG.GetTailState()["frequency"] == 0? 0: -1);

            AddReward(training.GetReward());
        }

        /// <summary>
        /// Provides heuristic/manual control for testing purposes by keeping all actions at zero.
        /// </summary>
        /// <param name="actionsOut">The action buffers to write heuristic actions to.</param>
        public override void Heuristic(in ActionBuffers actionsOut)
        {
            if (waiting > 0) {
                return;
            }

            // Provide manual control for testing purposes
            var continuousActionsOut = actionsOut.ContinuousActions;
            var discreteActionsOut = actionsOut.DiscreteActions;

            if (training.Config["Swimming"])
            {
                // Phase shifts
                for (int i = 0; i < 4; i++)                             continuousActionsOut[i] = -1;
                // spine phase
                for (int i = 4; i < 6; i++)                             continuousActionsOut[i] = -1;
                // amplitude change
                for (int i = 6; i < 10; i++)                            continuousActionsOut[i] = 0f;
                // spine amplitudes
                continuousActionsOut[10] = 0f;                                  // pitch
                continuousActionsOut[11] = 0f;                                  // spine yaw
                // drection change
                for (int i = 12; i < 16; i++)                           continuousActionsOut[i] = 0f;
                
                continuousActionsOut[continuousActionsOut.Length-1] = 0f;       // buoyancy
                discreteActionsOut[0] = 2;                                      // tail amp
                discreteActionsOut[1] = 1;                                      // tail freq
            }
            else
            {
                // Phase shifts
                for (int i = 0; i < 4; i++)                             continuousActionsOut[i] = -0.2f;
                // spine phase
                for (int i = 4; i < 6; i++)                             continuousActionsOut[i] = -0.2f;
                // amplitude change
                for (int i = 6; i < 10; i++)                            continuousActionsOut[i] = 0f;
                // spine amplitudes
                continuousActionsOut[10] = -1f;                                 // pitch
                continuousActionsOut[11] = 1f;                                  // yaw
                // drection change
                for (int i = 12; i < 16; i++)                           continuousActionsOut[i] = 0f;
                
                continuousActionsOut[continuousActionsOut.Length-1] = 0f;      // buoyancy
                discreteActionsOut[0] = 1;                                      // tail amp
                discreteActionsOut[1] = 0;                                      // tail freq
            }
        }
    }
}