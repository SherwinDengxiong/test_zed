using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class RaycasterButton : MonoBehaviour
{
    public UnityEvent events;

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void Invoke()
    {
        events.Invoke();
    }
}
