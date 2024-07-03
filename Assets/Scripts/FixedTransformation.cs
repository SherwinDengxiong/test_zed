using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FixedTransformation : MonoBehaviour
{

    public Vector3 targetLocalPosition = new Vector3(0, 0, 0);
    public Vector3 targetLocalRotation = new Vector3(0, 0, 0);
    public Vector3 targetLocalScale = new Vector3(1, 1, 1);

    private void LateUpdate()
    {
        transform.localPosition = targetLocalPosition;
        transform.localRotation = Quaternion.Euler(targetLocalRotation);
        transform.localScale = targetLocalScale;
    }
}
