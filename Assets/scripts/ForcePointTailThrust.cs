using UnityEngine;

namespace RoboIguanaAgentRL {
    /// <summary>
    /// Simple Force point to apply forces to an <c>ArticulationBody</c>.
    /// </summary>
    public class ForcePointTailThrust : MonoBehaviour
    {
        [Header("ArticulationBody")]
        public ArticulationBody body;

        Vector3 worldPostion;

        /// <summary>
        /// Applies a force to the <c>ArticulationBody</c> in the position of the gameObject.
        /// </summary>
        /// <param name="localForce"></param>
        public void ApplyForce (Vector3 localForce) 
        {
            worldPostion = transform.position;
            Vector3 worldForce = transform.TransformDirection(localForce);
            body.AddForceAtPosition(worldPostion, worldForce);
        }

    }
}