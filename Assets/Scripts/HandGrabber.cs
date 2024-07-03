using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandGrabber : MonoBehaviour
{
    public string objectTag = "GrabbableObject";

    private GameObject grabbedObject;
    private List<GameObject> hoverObjects = new List<GameObject>();

    void Start()
    {
        
    }

    void Update()
    {
        if (OVRInput.GetDown(OVRInput.Button.SecondaryHandTrigger) && hoverObjects.Count > 0)
        {
            grabbedObject = hoverObjects[0];
            grabbedObject.transform.SetParent(transform);
        }
        if (OVRInput.GetUp(OVRInput.Button.SecondaryHandTrigger) && hoverObjects.Count > 0 && grabbedObject != null)
        {
            grabbedObject.transform.SetParent(null);
            grabbedObject = null;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag.Equals(objectTag))
        {
            hoverObjects.Add(other.gameObject);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.tag.Equals(objectTag))
        {
            hoverObjects.Remove(other.gameObject);
        }
    }

}
