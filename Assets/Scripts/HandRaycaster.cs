using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HandRaycaster : MonoBehaviour
{
    public LineRenderer lineRenderer;
    public LayerMask selectLayers;


    private RaycasterMovableObject selectedObject;
    private float holdingDistance;
    public float moveObjectSpeed;

    void Start()
    {
        OVRManager.display.RecenterPose();
    }

    void Update()
    {
        //Debug.LogWarning(OVRInput.GetDown(OVRInput.Button.SecondaryIndexTrigger));
        
        //Raycast
        RaycastHit hit; // hit info
        if(Physics.Raycast(this.transform.position,this.transform.forward, out hit, 100, selectLayers.value))
        {
            //Debug.Log("Hit UI");
            if (OVRInput.GetDown(OVRInput.Button.SecondaryIndexTrigger)) // if right index trigger is pressed down
            {
                // if hit UI
                if (hit.collider.gameObject.GetComponent<RaycasterButton>() != null)
                {
                    hit.collider.gameObject.GetComponent<RaycasterButton>().Invoke();
                }
                // if hit movable object
                if (hit.collider.gameObject.GetComponent<RaycasterMovableObject>() != null)
                {
                    if(selectedObject == null) // if haven't selected any object
                    {
                        selectedObject = hit.collider.gameObject.GetComponent<RaycasterMovableObject>();
                        if (selectedObject.isMovable) // if the object is movable, select this object
                        {
                            selectedObject.pressEvent.Invoke();
                            selectedObject.InvokeFirstPressEvent();
                            holdingDistance = Vector3.Distance(selectedObject.transform.position, transform.position);
                        }
                        else
                        {
                            selectedObject = null;
                        }
                    }
                }
            }
        }
        if (OVRInput.GetUp(OVRInput.Button.SecondaryIndexTrigger) && selectedObject != null) // release the object
        {
            selectedObject.releaseEvent.Invoke();
            selectedObject = null;
        }
        if(selectedObject != null) // move the object based on the controller
        {
            selectedObject.transform.position = transform.position + transform.forward * holdingDistance;
            holdingDistance += OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick).y * Time.deltaTime * moveObjectSpeed;
            if(holdingDistance < 0)
            {
                holdingDistance = 0;
            }
        }
    }
}
