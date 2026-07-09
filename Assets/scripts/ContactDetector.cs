using UnityEngine;

/// <summary>
/// Detects contact between the game object and other colliders.
/// Tracks whether the object is currently touching the ground.
/// </summary>
public class ContactDetector : MonoBehaviour
{
    /// <summary>
    /// Indicates whether this object is currently touching the ground.
    /// </summary>
    public bool IsTouchingGround = false;

    /// <summary>
    /// Called when a collision begins between this collider and another collider.
    /// Sets IsTouchingGround to true.
    /// </summary>
    /// <param name="collision">The collision information.</param>
    void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.gameObject.name == "Floor") IsTouchingGround = true; 
    }

    /// <summary>
    /// Called when a collision ends between this collider and another collider.
    /// Sets IsTouchingGround to false.
    /// </summary>
    /// <param name="collision">The collision information.</param>
    void OnCollisionExit(Collision collision)
    {
        if (collision.collider.gameObject.name == "Floor") IsTouchingGround = false;
    }

    /// <summary>
    /// Resets the contact state by setting IsTouchingGround to false.
    /// </summary>
    public void Reset()
    {
        IsTouchingGround = false;
    }
}