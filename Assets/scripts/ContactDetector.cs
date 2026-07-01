using UnityEngine;

public class ContactDetector : MonoBehaviour
{
    public bool IsTouchingGround = false;

    void OnCollisionEnter(Collision collision)
    {
        IsTouchingGround = true;        
    }

    void OnCollisionExit(Collision collision)
    {
        IsTouchingGround = false;
    }

    void Reset()
    {
        IsTouchingGround = false;
    }
}