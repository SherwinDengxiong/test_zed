using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class RaycasterMovableObject : MonoBehaviour
{
    public UnityEvent firstPressEvent;
    public UnityEvent pressEvent;
    public UnityEvent releaseEvent;

    public bool isMovable = true;
    private bool isFirstPress = true;

    void Start()
    {
        
    }

    void Update()
    {
    }

    public void SetMovable(bool flag)
    {
        isMovable = flag;
    }

    public void InvokeFirstPressEvent()
    {
        if (isFirstPress) // if it is first press
        {
            firstPressEvent.Invoke();
            isFirstPress = false;
        }
    }

    public void ResetFirstPress()
    {
        isFirstPress = true;
    }
}
