using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FakeArm : MonoBehaviour
{
    public GameObject target;
    public float distance;
    public float lerpK;
    public OVRInput.Controller controller;
    public GameObject controllerObject;

    [HideInInspector] public bool isMoving = true;

    private Vector3 targetPosition;
    private Quaternion targetRotation;
    
    private bool isTrackOrientation = false;


    void Start()
    {
        CalculateTransform();
        transform.position = Vector3.Lerp(transform.position, targetPosition, lerpK);
        isTrackOrientation = false;
        //transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, lerpK);
    }

    void Update()
    {
        
        if (isMoving)
        {
            CalculateTransform();
            transform.position = Vector3.Lerp(transform.position, targetPosition, lerpK);

            if (OVRInput.GetDown(OVRInput.Button.PrimaryIndexTrigger))
            {
                isTrackOrientation = !isTrackOrientation;
            }
            if (isTrackOrientation == true)
            {
                transform.rotation = Quaternion.Lerp(transform.rotation, targetRotation, lerpK);
            }
        }

        //transform.rotation = Quaternion.Euler(new Vector3(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, controllerObject.transform.rotation.eulerAngles.z));

        //use transform.position to control
        //use transform.rotation to control

    }

    public void CalculateTransform()
    {
        
        Vector3 direction = (transform.position - target.transform.position).normalized;
        float dis = Vector3.Distance(target.transform.position, transform.position);
        if(dis > distance)
        {
            targetPosition = target.transform.position + direction * distance;
        }

        //targetRotation = OVRInput.GetLocalControllerRotation(controller);

        

        targetRotation = Quaternion.LookRotation(-direction, controllerObject.transform.up);
        //transform.LookAt(target.transform.position);
    }
}
