using UnityEngine;

namespace RoboIguanaAgentRL {
    /// <summary>
    /// Simple Force point to apply forces to an <c>ArticulationBody</c>.
    /// </summary>
    public class SimpleForcePoint : MonoBehaviour
    {
        [Header("ArticulationBody")]
        public ArticulationBody body;

        Vector3 worldPostion;

        /// <summary>
        /// Applies a force to the <c>ArticulationBody</c> in the position of the gameObject.
        /// </summary>
        /// <param name="localForce"></param>
        public void ApplyLocalForce (Vector3 localForce) 
        {
            Vector3 worldForce = transform.TransformDirection(localForce);
            ApplyWorldForce(worldForce);
        }

        public void ApplyWorldForce(Vector3 worldForce)
        {
            worldPostion = transform.position;
            body.AddForceAtPosition(worldPostion, worldForce);
        }

    }
}