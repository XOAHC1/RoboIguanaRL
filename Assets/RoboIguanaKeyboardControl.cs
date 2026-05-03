using UnityEngine;
using System.Collections.Generic;
using System.IO;
using Hinge = VehicleComponents.Actuators.Hinge;

namespace SmarcGUI.KeyboardControllers
{
    public class RoboIguanaIKController : KeyboardControllerBase
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

        [Header("Phase shift")]
        public float PhaseShift;

        [Header("Spine amplitude (deg)")]
        [Range(0f, 20f)]
        public float spineAmplitudeDeg = 0f;

        [Header("Spine stiffness")]
        public float spineStiffness = 500f;

        // =========================================================
        // TIMING
        // =========================================================

        [Header("Gait timing")]
        public float frequencyHz = 0.2f;
        public float startDelay = 5f;

        float time;

        // =========================================================
        // LEG GEOMETRY
        // =========================================================

        [Header("Leg geometry (meters)")]
        public float a = 0.116f;
        public float b = 0.09f;
        public float c = 0.172f;
        public float d = 0.2f;

        // =========================================================
        // FOOT TRAJECTORY
        // =========================================================

        [Header("Foot trajectory")]
        public Vector3 centerRight = new Vector3(-0.07f, -0.18f, -0.25f);
        public Vector3 centerLeft = new Vector3(0.07f, -0.18f, -0.25f);

        public float Rx = 0.08f;
        public float Ry = 0.04f;

        // =========================================================
        // INITIALISE JOINT
        // =========================================================

        void InitialiseJoint(ArticulationBody ab, Hinge h, float angleRad)
        {
            ab.jointPosition = new ArticulationReducedSpace(angleRad);
            var drive = ab.xDrive;
            drive.target = angleRad;
            ab.xDrive = drive;
            h.SetAngle(angleRad);
        }

        // =========================================================
        // INITIAL POSE
        // =========================================================

        void Start()
        {
            time = 0f;

            float phaseA = PhaseShift * Mathf.PI;
            float phaseB = Mathf.PI + PhaseShift * Mathf.PI;

            SolveAngles(
                phaseA, phaseB, Ry,
                out float yawA, out float hipA, out float kneeA,
                out float yawB, out float hipB, out float kneeB
            );

            InitialiseJoint(hyFL_Link, hyFL, Mathf.PI + yawA);
            InitialiseJoint(hyRR_Link, hyRR, -Mathf.PI - yawA);
            InitialiseJoint(hyFR_Link, hyFR, -Mathf.PI - yawB);
            InitialiseJoint(hyRL_Link, hyRL, Mathf.PI + yawB);

            InitialiseJoint(hpFL_Link, hpFL, hipA);
            InitialiseJoint(hpRR_Link, hpRR, hipA);
            InitialiseJoint(hpFR_Link, hpFR, hipB);
            InitialiseJoint(hpRL_Link, hpRL, hipB);

            InitialiseJoint(kFL_Link, kFL, kneeA);
            InitialiseJoint(kRR_Link, kRR, kneeA);
            InitialiseJoint(kFR_Link, kFR, kneeB);
            InitialiseJoint(kRL_Link, kRL, kneeB);

            // ---------- spine ----------
            // ---------- spine ----------
            if (spine != null && spine_Link != null)
            {
                // ✅ SAFE: no compile-time dependency
                var estimator = spine_Link.GetComponent("ServoPowerEstimator") as MonoBehaviour;

                if (estimator != null)
                {
                    if (spineAmplitudeDeg< 0.5f ||
                        spine_Link.jointType == ArticulationJointType.FixedJoint)
                    {
                        estimator.enabled = false;
                        Debug.Log("Spine estimator DISABLED");
                    }
                    else
                    {
                        estimator.enabled = true;
                    }
                }

                if (spine_Link.jointType == ArticulationJointType.RevoluteJoint)
                {
                    var drive = spine_Link.xDrive;
                    drive.stiffness = spineStiffness;
                    spine_Link.xDrive = drive;

                    float initialSpineAngle = spineAmplitudeDeg * Mathf.Deg2Rad;
                    InitialiseJoint(
                        spine_Link,
                        spine,
                        (-2f * PhaseShift + 1f) * initialSpineAngle
                    );
                }
            }
        }

        // =========================================================
        // UPDATE
        // =========================================================

        void FixedUpdate()
        {
            time += Time.fixedDeltaTime;

            if (time < startDelay)
                return;

            float gaitTime = time - startDelay;
            float T = 1f / frequencyHz;

            float localTime = gaitTime % T;

            float phaseA = 2f * Mathf.PI * localTime / T;
            float phaseB = phaseA + Mathf.PI;

            phaseA += PhaseShift * Mathf.PI;
            phaseB += PhaseShift * Mathf.PI;

            SolveAndApply(phaseA, phaseB, Ry);

            // spine
            if (spine != null && spine_Link != null &&
                spine_Link.jointType == ArticulationJointType.RevoluteJoint)
            {
                float A = spineAmplitudeDeg * Mathf.Deg2Rad;
                float spineAngleRad =
                    (-2f * PhaseShift + 1f) * A *
                    Mathf.Cos(2f * Mathf.PI * gaitTime / T);

                spine.SetAngle(spineAngleRad);
            }
        }

        // =========================================================
        // IK SOLVER
        // =========================================================

        void SolveAndApply(float phaseA, float phaseB, float ry)
        {
            SolveAngles(
                phaseA, phaseB, ry,
                out float yawA, out float hipA, out float kneeA,
                out float yawB, out float hipB, out float kneeB
            );

            ApplyAngles(yawA, hipA, kneeA, yawB, hipB, kneeB);
        }

        void SolveAngles(
            float phaseA, float phaseB, float ry,
            out float yawA, out float hipA, out float kneeA,
            out float yawB, out float hipB, out float kneeB)
        {
            Vector3 pRight = centerRight +
                new Vector3(Rx * Mathf.Cos(phaseA),
                            ry * Mathf.Sin(phaseA), 0);

            Vector3 pLeft = centerLeft +
                new Vector3(Rx * Mathf.Cos(phaseB),
                            ry * Mathf.Sin(phaseB), 0);

            InverseKinematics(pRight, a, b, c, d,
                out yawA, out hipA, out kneeA);

            InverseKinematics(pLeft, a, b, c, d,
                out yawB, out hipB, out kneeB);
        }

        void ApplyAngles(
            float yawA, float hipA, float kneeA,
            float yawB, float hipB, float kneeB)
        {
            hyFL.SetAngle(Mathf.PI + yawA);
            hyRR.SetAngle(-Mathf.PI - yawA);
            hyFR.SetAngle(-Mathf.PI - yawB);
            hyRL.SetAngle(Mathf.PI + yawB);

            hpFL.SetAngle(hipA);
            hpRR.SetAngle(hipA);
            hpFR.SetAngle(hipB);
            hpRL.SetAngle(hipB);

            kFL.SetAngle(kneeA);
            kRR.SetAngle(kneeA);
            kFR.SetAngle(kneeB);
            kRL.SetAngle(kneeB);
        }

        // =========================================================
        // IK MATH
        // =========================================================

        void InverseKinematics(
            Vector3 p,
            float a, float b, float c, float d,
            out float theta1,
            out float theta3,
            out float theta4)
        {
            float x = p.x;
            float y = p.y;
            float z = p.z;

            float r2 = x * x + z * z;
            float Lmag = Mathf.Sqrt(Mathf.Max(0f, r2 - b * b));

            theta1 = Mathf.Atan2(z, -x);

            float Xp = Lmag - a;
            float Yp = -y;

            float D = (Xp * Xp + Yp * Yp - c * c - d * d) / (2f * c * d);
            D = Mathf.Clamp(D, -1f, 1f);

            theta4 = Mathf.Atan2(Mathf.Sqrt(1 - D * D), D);

            theta3 = Mathf.Atan2(Yp, Xp) -
                     Mathf.Atan2(d * Mathf.Sin(theta4),
                                 c + d * Mathf.Cos(theta4));
        }

        public override void OnReset()
        {
            time = 0f;
        }
    }
}