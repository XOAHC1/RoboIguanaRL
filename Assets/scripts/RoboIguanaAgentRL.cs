using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using System.Linq;
using System.Collections.Generic;
using UnityEngine.SceneManagement;
using Unity.VisualScripting;

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
        public FootForceSensor footFL, footFR, footRL, footRR;

        /// <summary>
        /// Point to observe robot position.
        /// </summary>
        public ArticulationBody obs;
        
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
        /// X is relative to the robot, whereas y is in absolute coordinates.
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
        private Vector3 higherPos, StartingPosition;

        /// <summary>
        /// Initial orientation of the robot.
        /// </summary>
        private Quaternion StartingOrientation;

        /// <summary>
        /// Contains <c>ArticulationBody</c> elements of all components of the robot.
        /// </summary>
        private List<ArticulationBody> ComponentABs;

        /// <summary>
        /// Number of physics steps to wait at the begin of an episode, to let the robot settle.
        /// </summary>
        private int waiting;
        private readonly int waitSteps = 150;

        /// <summary>
        /// Number of agent decisions until new target inputs are generated.
        /// </summary>
        private int nextTargetSteps, nextTargetFreq = 250;

        /// <summary>
        /// Number of target resets until locomotion mode is changed when landing.
        /// </summary>
        private int 
            nextLocomotionmode = 2, 
            locomotionModeChange= 2;

        private float fixedHeight;

        private bool firstEpisode;

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
            ComponentABs = GetComponentsInChildren<ArticulationBody>().ToList();
            ComponentABs.Add(Body);;

            training = new TrainingManager();

            // save starting parameters
            transform.GetPositionAndRotation(out StartingPosition, out StartingOrientation);
            StartingPosition.y += 0.01f;
            var startHeight = training.Config["Swimming"]? 1.5f: 0.5f;
            higherPos = new Vector3(StartingPosition.x, StartingPosition.y+startHeight, StartingPosition.z);

            // apply settings
            MaxStep = training.Config["LongEpisodes"]? 20000: 10000;
            if (training.Config["2D"]) fixedHeight = training.Config["Swimming"]? higherPos.y: StartingPosition.y;
            firstEpisode = true;

            Debug.Log("Agent initialization over");
        }

        /// <summary>
        /// Resets the Robots Positon, CPG and Sensors.
        /// </summary>
        public void ResetRobot()
        {
            if (firstEpisode) firstEpisode = false;
            else {SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);}
            waiting = waitSteps;

            // Reset Robot Position
            CPG.DoReset(training.Config["RandomStart"]);

            if (training.Config["Swimming"] || training.Config["Landing"]) Body.TeleportRoot(higherPos, StartingOrientation);

            foreach (ArticulationBody ab in ComponentABs)
            {
                ab.linearVelocity = Vector3.zero;
                ab.angularVelocity = Vector3.zero;
                // drives
                var xDrive = ab.xDrive;
                xDrive.target = 0f;
                ab.xDrive = xDrive;

            }

            // Reset foot contact sensors
            footFL.Reset();
            footFR.Reset();
            footRL.Reset();
            footRR.Reset();
            Back.Reset();
        }

        public new void EndEpisode()
        {
            Debug.Log("Episode ended");

            base.EndEpisode();
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

            // set locomotion mode to start value
            if (training.Config["Transition"]) {training.Config["Swimming"] = true; training.Config["Landing"] = false;}

            training.NewEpisode();
            if (!firstEpisode && training.LogHistory) training.LogEpisode();

            ResetTarget();
            nextLocomotionmode = locomotionModeChange;

            ResetRobot();

            SetReward(0f);
        }

        /// <summary>
        /// Collects state observations and adds them to a VectorSensor.
        /// </summary>
        /// <remarks>
        /// Observed are:
        ///     World State:
        ///         Locomotion type                 [0, 1]
        ///         Linear velocity error           2D
        ///         Target angular velocity         2D
        ///         angular velocty                 3D
        ///         Ground contact booleans         4D
        ///     CPG State:
        ///         Phases                          5D
        ///         Phase shifts                    5D
        ///         Amplitudes                      5D
        ///         Ampltude shifts                 5D
        ///         Orientation offsets             4D
        ///         Orientation offset shifts       4D
        ///     Others:
        ///         Spine pitch                     2D
        ///         Buoyancy                        2D
        ///         Tail State                      3D
        /// For a total of 47 input dimensions.
        /// </remarks>
        /// <param name="sensor">The vector sensor to add observations to.</param>
        public override void CollectObservations(VectorSensor sensor)
        {
            if (training.Config["Analysis"]){
                Debug.Log($"Linear velocity: {obs.linearVelocity.y - 0.14f} \n Angular velocity: {obs.transform.InverseTransformDirection(obs.angularVelocity)} \n Robot Position: {obs.transform.position}");
                }
            if (nextTargetSteps < 2) ResetTarget();
            else nextTargetSteps --;

            if (!training.Config["Swimming"])
                // TargetLinearVelocity.y = Mathf.Clamp(-Mathf.Pow((obs.transform.position.y - StartingPosition.y) / (higherPos.y-StartingPosition.y), 2), -0.2f, 0);
                TargetLinearVelocity.y = Mathf.Clamp(-(obs.transform.position.y - StartingPosition.y) * 1.5f, -0.2f, -0.005f);

            var relTarget = obs.transform.InverseTransformDirection(TargetLinearVelocity);
            var actVel = obs.linearVelocity; actVel.y -= 0.14f;
            var relObserv = obs.transform.InverseTransformDirection(actVel);

            // Debug.Log($"Target: {TargetLinearVelocity}, ObsVel: {obs.linearVelocity}, act_vel {actVel}");

            Debug.Log($"Target: ({TargetLinearVelocity.x}, {TargetLinearVelocity.y}), actual: {actVel}");

            // position and velocity observations
            sensor.AddObservation(locomotionType);
            sensor.AddObservation(TargetLinearVelocity.x - relObserv.x);
            sensor.AddObservation(actVel.y - TargetLinearVelocity.y);
            sensor.AddObservation(TargetAngularVelocity);
            sensor.AddObservation(transform.InverseTransformDirection(obs.angularVelocity));
            sensor.AddObservation(obs.transform.up);

            // Contact Booleans
            sensor.AddObservation(footFR.verticalForce);
            sensor.AddObservation(footFL.verticalForce);
            sensor.AddObservation(footRL.verticalForce);
            sensor.AddObservation(footRR.verticalForce);

            // internal state
            sensor.AddObservation(CPG.GetPhases());
            sensor.AddObservation(CPG.GetPhaseShifts());
            sensor.AddObservation(CPG.GetAmplitudes());
            sensor.AddObservation(CPG.GetAmplitudeShifts());
            sensor.AddObservation(CPG.GetOrientationOffsets());
            sensor.AddObservation(CPG.GetOrientationOffsetShifts());
            sensor.AddObservation(CPG.GetSpinePitchState());

            // Buoyancy
            sensor.AddObservation(CPG.GetBuoyancyState());

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
        ///         change intrinsic frequency  1D
        ///         change amplitude            1D
        //          spine pitch target          1D
        ///     buoyancy:
        ///         change in buoyancy          1D
        ///   discrete:
        ///     Tail:                          
        ///         yaw amplitude               [0, 1, 2]   Will be translated to [-1, 0, 1] later on.
        ///         frequency                   [0, 1]      Tail off/on
        ///         
        /// For a total of 18 action dimensions.
        /// </remarks>
        /// <param name="buffers">The action buffers containing the policy decisions.</param>
        public override void OnActionReceived(ActionBuffers buffers)
        {
            if (training.Config["Analysis"])
                Debug.Log($"Agent Actons: Continuous=[{string.Join(", ", buffers.ContinuousActions.ToArray())}], Discrete=[{string.Join(", ", buffers.DiscreteActions.ToArray())}]");
            
            var cont = buffers.ContinuousActions;
            var disc = buffers.DiscreteActions;

            training.LinRewards["SimpleTrainingPenalties"] = 0;

            // block and punish undesirable actions in training mode

            // Leg Control
            if (!training.Config["LegPhases"]) for (int i = 0; i<4; i++) {cont[i] = -1; training.LinRewards["SimpleTrainingPenalties"] += (cont[i] + 1)/2;}
            if (!training.Config["LegAmplitudes"]) for (int i=5;i<9;i++) {cont[i] = 0; training.LinRewards["SimpleTrainingPenalties"] += Mathf.Abs(cont[i]);}
            if (!training.Config["LegRotations"]) for (int i=10;i<14;i++) {cont[i] = 0; training.LinRewards["SimpleTrainingPenalties"] += Mathf.Abs(cont[i]);}

            // Spine Control
            if (!training.Config["SpinePhases"]) for (int i=4;i<5;i++) {cont[i] = -1; training.LinRewards["SimpleTrainingPenalties"] += (cont[i] + 1)/2;}
            if (!training.Config["SpineAmplitudes"]) for (int i=9;i<10;i++) {cont[i] = 0; training.LinRewards["SimpleTrainingPenalties"] += Mathf.Abs(cont[i]);}
            if (!training.Config["SpinePitch"]) {cont[14] = 0f; training.LinRewards["SimpleTrainingPenalties"] += Mathf.Abs(cont[14]);}

            // buoyancy
            if (!training.Config["Buoyancy"])  {cont[15] = -1; training.LinRewards["SimpleTrainingPenalties"] += (cont[15] + 1)/2;}

            // Tail
            if (!training.Config["Tail"]) for (int i=0;i<2;i++) {disc[i] = 0; training.LinRewards["SimpleTrainingPenalties"] += disc[i]+1f;}


            // block buoyancy for landing mode
            if (training.Config["Landing"])
            {
                // punish
                training.LinRewards["SimpleTrainingPenalties"] += 
                    cont[15] > 0? cont[15]: 0;
                // block
                // cont[15] = Mathf.Clamp(cont[15], -1, 0.5f);
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

            // update locomotion mode
            if(training.Config["Transition"] && (training.Config["Swimming"] || training.Config["Landing"]))
            {
                // Debug.Log($"modeCounter: {nextLocomotionmode}");
                if (nextLocomotionmode == 0)
                {
                    if (training.Config["Swimming"])
                    {
                        Debug.Log("Start landing");
                        training.Config["Swimming"] = false;
                        training.Config["Landing"] = true;
                    }
                    else if (training.Config["Landing"])
                    {
                        Debug.Log("Finished landing");
                        training.Config["Landing"] = false;
                    }
                    nextLocomotionmode = locomotionModeChange;
                }
                else nextLocomotionmode--;
            }

            // settle locomotion type
            locomotionType = training.Config["Swimming"]? 0: training.Config["Landing"]? 2: 1;

            // generate target velocities, foreward and upward
            TargetLinearVelocity = new Vector2(
                training.Config["RandomXVelocity"] ? Random.Range(0.02f, 0.25f): 0.15f,
                training.Config["RandomYVelocity"] && training.Config["Swimming"]? Random.Range(-0.2f, 0.2f): 0f
            );
            
            // generate target angular velocities
            TargetAngularVelocity = training.Config["RandomAngularVelocity"] ?
                // random values
                new Vector2 (
                    Random.Range(-0.3f, 0.3f),
                    (locomotionType == 0) ? 
                        Random.Range(-0.2f, 0.2f):
                        (locomotionType == 2)? -0.1f :
                        0f
                ): 
                // default values
                new Vector2 (
                    0f,
                    0f
                );
            
            if (training.Config["Analysis"]) 
                Debug.Log($"New Target: \n LinVel: {TargetLinearVelocity} \n AngVel: {TargetAngularVelocity}");
        }

        /// <summary>
        /// Terminates Episode in unsalvagable situations. Applies reward.
        /// </summary>
        public void FixedUpdate()
        {
            
            if (training.Config["2D"]) {
                var p= transform.position; p.y = fixedHeight; transform.position = p; 
                var v= Body.linearVelocity; v.y = 0; Body.linearVelocity = v;
                }
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
            // TerminateIfNecessary();
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
            bool groundContact = footFL.contact || footFR.contact || footRL.contact || footRR.contact;

            // precalculate velocites
            var relTarLinVel = obs.transform.InverseTransformDirection(TargetLinearVelocity);
            var relVel = obs.transform.InverseTransformDirection(obs.linearVelocity);
            var relAngVel = obs.transform.InverseTransformDirection(obs.angularVelocity);

            // linear velocity x
            training.ExpRewards["xVel"] = (relVel.x - TargetLinearVelocity.x) / ((TargetLinearVelocity.x != 0)? Mathf.Abs(TargetLinearVelocity.x): 0.01f);
            // linear velocity y
            training.ExpRewards["yVel"] = (obs.linearVelocity.y - TargetLinearVelocity.y) / ((TargetLinearVelocity.y != 0)? Mathf.Abs(TargetLinearVelocity.y): 0.01f);
            // linear velocity z
            training.QuadPenalties["zVel"] = relVel.z;
            // angular velocity roll
            training.QuadPenalties["rollRate"] = relAngVel.x;
            // angular velocity yaw
            training.ExpRewards["yawRate"] = (relAngVel.y - TargetAngularVelocity.x ) / ((TargetAngularVelocity.x != 0)? Mathf.Abs(TargetAngularVelocity.x): 0.01f);
            // angular velocity pitch
            training.ExpRewards["pitchRate"] = (relAngVel.z - TargetAngularVelocity.y) /((TargetAngularVelocity.y != 0)? Mathf.Abs(TargetAngularVelocity.y): 0.01f);
            // Work
            training.QuadPenalties["work"] = EnergyEstimator.CurrentEnergy;
            // ground contact
            training.LinRewards["groundContact"] = ((locomotionType == 1) ? 1: (locomotionType == 2)? 0: -1) * (groundContact ? 1f : -1f);
            // Tail Status
            training.LinRewards["tailStatus"] = ((locomotionType == 1) ? 1: 0) * (CPG.GetTailState()["frequency"] == 0? 0: -1);
            // swimm height
            training.ExpRewards["yPos"] = (Body.transform.position.y - higherPos.y)/(StartingPosition.y-higherPos.y);
            // orientation of the robot
            training.ExpRewards["orientation"] = (obs.transform.up - Vector3.up).magnitude / 2;

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
                for (int i = 4; i < 5; i++)                             continuousActionsOut[i] = -1;
                // amplitude change
                for (int i = 5; i < 9; i++)                            continuousActionsOut[i] = 0f;
                // spine amplitude
                continuousActionsOut[9] = 0f;                                  // spine yaw
                // drection change
                for (int i = 10; i < 13; i++)                          continuousActionsOut[i] = 0f;
                
                continuousActionsOut[14] = -1;                                   // spine pitch
                continuousActionsOut[continuousActionsOut.Length-1] = 0f;       // buoyancy
                discreteActionsOut[0] = 2;                                      // tail amp
                discreteActionsOut[1] = 1;                                      // tail freq

                // For Testing
                // for (int i = 0; i < 17; i++) continuousActionsOut[i] = 0;
                // for (int i = 0; i < 2; i++) discreteActionsOut[i] = 0;
            }
            else if (training.Config["Landing"])
            {
                // Phase shifts
                for (int i = 0; i < 4; i++)                             continuousActionsOut[i] = -0.2f;
                // spine phase
                for (int i = 4; i < 5; i++)                             continuousActionsOut[i] = -0.2f;
                // amplitude change
                for (int i = 5; i < 9; i++)                            continuousActionsOut[i] = 0f;
                // spine amplitudes
                continuousActionsOut[9] = 0f;                                 // yaw
                // drection change
                for (int i = 10; i < 13; i++)                           continuousActionsOut[i] = 0f;
                
                continuousActionsOut[14] = 0;                                   // spine pitch
                continuousActionsOut[continuousActionsOut.Length-1] = -1f;      // buoyancy
                discreteActionsOut[0] = 0;                                      // tail amp
                discreteActionsOut[1] = 1;                                      // tail freq
            }
            else
            {
                // Phase shifts
                for (int i = 0; i < 4; i++)                             continuousActionsOut[i] = -0f;
                // spine phase
                for (int i = 4; i < 5; i++)                             continuousActionsOut[i] = -0f;
                // amplitude change
                for (int i = 5; i < 9; i++)                            continuousActionsOut[i] = 0f;
                // spine amplitudes
                continuousActionsOut[9] = 1f;                                  // yaw
                // drection change
                for (int i = 10; i < 13; i++)                           continuousActionsOut[i] = 0f;
                
                continuousActionsOut[14] = 0;                                   // spine pitch
                continuousActionsOut[continuousActionsOut.Length-1] = -1f;      // buoyancy
                discreteActionsOut[0] = 1;                                      // tail amp
                discreteActionsOut[1] = 0;                                      // tail freq
            }
        }
    }
}