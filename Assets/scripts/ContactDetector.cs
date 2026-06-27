using UnityEngine;

public class ContactDetector : MonoBehaviour

    public bool isInContact = false;
{
    void OnCollisionEnter(Collision collision)
    {
        isInContact = true;        
    }

    void OnCollisionExit(Collision collision)
    {
        isInContact = false;
    }

    void Reset()
    {
        isInContact = false;
    }
}