using UnityEngine;

namespace RoboIguanaRL {

    /// <summary>
    /// Simple Force point to apply forces to an <c>ArticulationBody</c>.
    /// </summary>
    public class SimpleForcePoint : MonoBehaviour
    {
        /// <summary>
        /// Body to which force is applied.
        /// </summary>
        [Header("ArticulationBody")]
        public ArticulationBody body;

        /// <summary>
        /// Applies a force relatve to the body to the <c>ArticulationBody</c>.
        /// </summary>
        /// <param name="localForce"></param>
        public void ApplyLocalForce (Vector3 localForce) 
        {
            body.AddRelativeForce(localForce);
        }

        /// <summary>
        /// Applies a force in world coordinates to the object.
        /// </summary>
        /// <param name="worldForce"></param>
        public void ApplyWorldForce(Vector3 worldForce)
        {
            body.AddForce(worldForce);
        }
    }
}