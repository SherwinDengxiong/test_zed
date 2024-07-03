using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIManager : MonoBehaviour
{
    public static UIManager instance;

    public GameObject indicator;


    private void Awake()
    {
        if(instance == null)
        {
            instance = this;
        }
        else
        {
            Destroy(this.gameObject);
        }
    }

    void Start()
    {
        
    }

    void Update()
    {
        
    }

    public void ShowIndicator()
    {
        indicator.SetActive(true);
    }

    public void HideIndicator()
    {
        indicator.SetActive(false);
    }

    public void Quit()
    {
        Application.Quit();
    }
}
