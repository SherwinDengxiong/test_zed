using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SetTransformation : MonoBehaviour
{
    public Vector3 position;
    public Vector3 rotation;
    public Vector3 scale;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void Reset()
    {
        transform.position = position;
        transform.rotation = Quaternion.Euler(rotation);
        transform.localScale = scale;
    }
}
