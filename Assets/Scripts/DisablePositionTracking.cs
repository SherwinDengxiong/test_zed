using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DisablePositionTracking : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        UnityEngine.XR.InputTracking.disablePositionalTracking = false;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
