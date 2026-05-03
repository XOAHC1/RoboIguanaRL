using UnityEngine;

public class ContactDetector : MonoBehaviour
{
    void OnCollisionEnter(Collision collision)
    {
        foreach (ContactPoint contact in collision.contacts)
        {
            Debug.Log("Contact point: " + contact.point);
            Debug.Log("Contact normal: " + contact.normal);
        }
    }
}