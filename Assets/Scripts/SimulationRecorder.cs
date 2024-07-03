using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class SimulationRecorder : MonoBehaviour
{
    public LineRenderer lineRender;
    public List<Vector3> recordedPositions = new List<Vector3>();
    public List<Quaternion> recordedRotations = new List<Quaternion>();
    public List<Quaternion> originalRecordedRotations = new List<Quaternion>();
    public bool gripper_change = false;

    public Text gripper_text;
    [HideInInspector] public bool gripper_status = true;

    public float recordInterval = 0.1f;

    public UnityEvent replayStartEvent;
    public UnityEvent replayEndEvent;
    public float replaySpeed = 1.0f;

    private bool isRecording;
    private float recordTimer = 0;

    private bool isReplaying;
    private float replayTimer = 0;
    private int replayingIndex = 0;

    private FakeArm arm;

    void Start()
    {
        arm = GetComponent<FakeArm>();
    }

    void Update()
    {

        
        if (isRecording)
        {
            if (OVRInput.GetDown(OVRInput.Button.One))
            {
                ChangeGripperStatus();
                gripper_change = true;//change gripper status, send mode 4
            }

            recordTimer += Time.deltaTime;
            if (recordTimer >= recordInterval)
            {
                recordedPositions.Add(transform.position); // record a position
                Quaternion tempQ = transform.rotation;
                //Quaternion tempQ = new Quaternion(transform.rotation.x, transform.rotation.z, transform.rotation.y, -transform.rotation.w);
                //Quaternion tempQ = new Quaternion(-transform.rotation.x, -transform.rotation.z, -transform.rotation.y, transform.rotation.w); // convert to right hand system

                //tempQ = Quaternion.Euler(tempQ.eulerAngles + new Vector3(90, 0, 0));
                tempQ *= Quaternion.Euler(90, 0, 0);
                recordedRotations.Add(tempQ);
                //recordedRotations.Add(transform.rotation);

                originalRecordedRotations.Add(transform.rotation);

                lineRender.positionCount++;
                lineRender.SetPosition(lineRender.positionCount - 1, transform.position);
                recordTimer = 0;
            }
        }
        if (isReplaying)
        {
            replayTimer += Time.deltaTime * replaySpeed;
            if(replayTimer >= recordInterval)
            {
                replayingIndex++;
                if (replayingIndex == recordedPositions.Count)
                {
                    EndReplay();
                }
                else
                {
                    lineRender.positionCount++; // reset the length of the array
                    lineRender.SetPosition(replayingIndex, recordedPositions[replayingIndex]); // add a new position into the linerenderer
                    this.transform.position = recordedPositions[replayingIndex]; // move the object to the latest position;
                    this.transform.rotation = originalRecordedRotations[replayingIndex];
                }
                replayTimer = 0;
            }
        }
    }

    public void StartRecord()
    {
        isRecording = true;
        recordedPositions.Clear();
        recordedRotations.Clear();
        originalRecordedRotations.Clear();
        lineRender.positionCount = 0;
    }

    public void ResumeRecord()
    {
        isRecording = true;
    }

    public void PauseRecord()
    {
        isRecording = false;
    }

    public void EndRecord()
    {
        isRecording = false;
    }

    public void StartReplay()
    {
        isReplaying = true;
        replayStartEvent.Invoke();
        replayingIndex = 0;
        replayTimer = 0;
        if(recordedPositions.Count == 0)
        {
            EndReplay();
            return;
        }
        this.transform.position = recordedPositions[0];
        lineRender.positionCount = 1;
        lineRender.SetPosition(0, recordedPositions[0]);
        arm.isMoving = false;
    }

    public void EndReplay()
    {
        isReplaying = false;
        replayEndEvent.Invoke();
        arm.isMoving = true;
    }

    public void ClearRecorder()
    {
        
        recordedPositions.Clear();
        recordedRotations.Clear();
        originalRecordedRotations.Clear();
    }

    public void ClearLines()
    {
        lineRender.positionCount = 0;
    }

    public void ChangeGripperStatus()
    {
        gripper_status = !gripper_status;
        if(gripper_status == true)
        {
            gripper_text.text = "Gripper is open";
        }
        else
        {
            gripper_text.text = "Gripper is closed";
        }
    }
}
