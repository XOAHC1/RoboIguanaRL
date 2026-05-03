using System;
using UnityEngine;

namespace Force
{
    public class ForcePointRoboIguana : MonoBehaviour
    {
        [Header("Connected Body (REQUIRED)")]
        public ArticulationBody ConnectedArticulationBody;
        public Rigidbody ConnectedRigidbody;

        [Header("Water Properties")]
        public float WaterDensity = 997f;

        [Header("Linear Drag (Local Axis Based)")]
        [Tooltip("Projected area along local X axis (m^2)")]
        public float AreaX = 0.01f;

        [Tooltip("Projected area along local Y axis (m^2)")]
        public float AreaY = 0.01f;

        [Tooltip("Projected area along local Z axis (m^2)")]
        public float AreaZ = 0.01f;

        [Tooltip("Drag coefficient along local X axis")]
        public float CdX = 1.2f;

        [Tooltip("Drag coefficient along local Y axis")]
        public float CdY = 1.2f;

        [Tooltip("Drag coefficient along local Z axis")]
        public float CdZ = 1.2f;

        [Header("Added Mass")]
        [Tooltip("Added mass acting along local X axis (kg)")]
        public float AddedMassX = 0.1f;

        [Tooltip("Added mass acting along local Y axis (kg)")]
        public float AddedMassY = 0.1f;

        [Tooltip("Added mass acting along local Z axis (kg)")]
        public float AddedMassZ = 0.1f;

        [Header("Force Limits")]
        private float MaxAddedMassForcePerAxis = 200f;

        private MixedBody body;

        private Vector3 previousLocalVelocity = Vector3.zero;
        private bool isFirstStep = true;

        void Awake()
        {
            body = new MixedBody(ConnectedArticulationBody, ConnectedRigidbody);

            if (!body.isValid)
            {
                Debug.LogWarning($"{name}: No valid connected body!");
                enabled = false;
                return;
            }
        }

        void FixedUpdate()
        {
            ApplyHydrodynamicForces();
        }

        void ApplyHydrodynamicForces()
        {
            Vector3 pos = transform.position;
            Vector3 worldVelocity = GetPointVelocity(pos);
            Vector3 localVelocity = transform.InverseTransformDirection(worldVelocity);

            //---------------------------------------
            // DRAG FORCE
            //---------------------------------------
            float Fx = -0.5f * WaterDensity * CdX * AreaX *
                       localVelocity.x * Mathf.Abs(localVelocity.x);

            float Fy = -0.5f * WaterDensity * CdY * AreaY *
                       localVelocity.y * Mathf.Abs(localVelocity.y);

            float Fz = -0.5f * WaterDensity * CdZ * AreaZ *
                       localVelocity.z * Mathf.Abs(localVelocity.z);

            Vector3 localDragForce = new Vector3(Fx, Fy, Fz);

            //---------------------------------------
            // ADDED MASS FORCE
            //---------------------------------------
            float dt = Time.fixedDeltaTime;
            Vector3 localAccelerationRaw = Vector3.zero;

            if (!isFirstStep && dt > 1e-6f)
            {
                localAccelerationRaw = (localVelocity - previousLocalVelocity) / dt;
            }

            float F_added_x = -AddedMassX * localAccelerationRaw.x;
            float F_added_y = -AddedMassY * localAccelerationRaw.y;
            float F_added_z = -AddedMassZ * localAccelerationRaw.z;

            F_added_x = Mathf.Clamp(F_added_x, -MaxAddedMassForcePerAxis, MaxAddedMassForcePerAxis);
            F_added_y = Mathf.Clamp(F_added_y, -MaxAddedMassForcePerAxis, MaxAddedMassForcePerAxis);
            F_added_z = Mathf.Clamp(F_added_z, -MaxAddedMassForcePerAxis, MaxAddedMassForcePerAxis);

            Vector3 localAddedMassForce = new Vector3(F_added_x, F_added_y, F_added_z);

            //---------------------------------------
            // TOTAL FORCE
            //---------------------------------------
            Vector3 localTotalForce = localDragForce + localAddedMassForce;
            Vector3 worldForce = transform.TransformDirection(localTotalForce);

            body.AddForceAtPosition(worldForce, pos);

            //---------------------------------------
            // STORE FOR NEXT STEP
            //---------------------------------------
            previousLocalVelocity = localVelocity;
            isFirstStep = false;
        }

        Vector3 GetPointVelocity(Vector3 worldPos)
        {
            if (ConnectedArticulationBody != null)
            {
                Vector3 r = worldPos - ConnectedArticulationBody.worldCenterOfMass;

                return ConnectedArticulationBody.linearVelocity +
                       Vector3.Cross(ConnectedArticulationBody.angularVelocity, r);
            }

            if (ConnectedRigidbody != null)
            {
                return ConnectedRigidbody.GetPointVelocity(worldPos);
            }

            return Vector3.zero;
        }
    }
}