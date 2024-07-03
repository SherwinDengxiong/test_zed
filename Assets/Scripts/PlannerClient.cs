using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.XarmMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class PlannerClient : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 6;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.001f;

    public SimulationRecorder recorder;
    public static readonly string[] LinkNames =
        { "world/link_base/link1", "/link2", "/link3", "/link4", "/link5", "/link6" };

    // Variables required for ROS communication
    [SerializeField]
    string m_RosServiceName = "xarm_mover";
    public string RosServiceName { get => m_RosServiceName; set => m_RosServiceName = value; }

    [SerializeField]
    GameObject m_Xarm;
    public GameObject Xarm { get => m_Xarm; set => m_Xarm = value; }
    [SerializeField]
    GameObject m_Target;
    public GameObject Target { get => m_Target; set => m_Target = value; }
    [SerializeField]
    GameObject m_TargetPlacement;
    public GameObject TargetPlacement { get => m_TargetPlacement; set => m_TargetPlacement = value; }

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;

    public List<Vector3> Positions = new List<Vector3>();
    public List<Quaternion> Rotations = new List<Quaternion>();


    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<TrajectoryServiceRequest, TrajectoryServiceResponse>(m_RosServiceName);

        Debug.LogWarning("service name : " + m_RosServiceName);
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {


            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_Xarm.transform.Find(linkName).GetComponent<ArticulationBody>();


        }

        // Find left and right fingers
        var rightGripper = linkName + "/link_eef/xarm_gripper_base_link/right_outer_knuckle/right_finger";
        var leftGripper = linkName + "/link_eef/xarm_gripper_base_link/left_outer_knuckle/left_finger";

        m_RightGripper = m_Xarm.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_Xarm.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>XarmMoveitJoints</returns>
    PoseListMsg CurrentJointConfig()
    {
        var pose_list = new PoseListMsg();

        Positions = recorder.recordedPositions;
        Rotations = recorder.recordedRotations;

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        Debug.LogWarning("position length :"+ Positions.Count);

        PoseMsg[] recorded_pose = new PoseMsg[Positions.Count];

        for (var i = 0; i < Positions.Count; i++)
        {
            var pose = new PoseMsg();
            pose.position = Positions[i].To<FLU>();
            if (i == 0)
            {
                //pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
                pose.orientation = Rotations[i].To<FLU>();

            }
            else {
                //pose.orientation = Quaternion.Euler(180,0,0).To<FLU>();
                pose.orientation = Rotations[i].To<FLU>();
            }
            //pose.orientation = Rotations[i].To<FLU>();

            Debug.LogWarning("element " + i + ":" + Rotations[i].eulerAngles);

            recorded_pose[i] = pose;

        }
        //recorded_pose[0] = new PoseMsg();
        //recorded_pose[0].position = new Vector3(0.4f, 0.0f, 0.0f).To<FLU>();
        //recorded_pose[0].orientation= Quaternion.Euler(180, 0, 0).To<FLU>();
        //Debug.LogWarning(recorded_pose[0].orientation);
        
        pose_list.pose_list = recorded_pose;

        if (recorder != null)
        {
            recorder.ClearRecorder();
        }

        Debug.LogWarning("recorder length :" + recorder.recordedPositions.Count);

        //var lastpose = pose_list.pose_list[Positions.Count - 1];

        



        return pose_list;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    /// 
    public void PublishJoints_0()
    {
        Debug.LogWarning("initialize mode 0 start, move to prepared pose");
        var request = new TrajectoryServiceRequest();
        var pose_list = new PoseListMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        PoseMsg[] recorded_pose = new PoseMsg[1];

        var pose = new PoseMsg();
        pose.position = new Vector3(0.8f, 0.0f, 0.0f).To<FLU>();
        pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        recorded_pose[0] = pose;
        pose_list.pose_list = recorded_pose;

        request.pose_list = pose_list;

        request.mode = 0;
        Debug.LogWarning("mode 0 sending!");
        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        Debug.LogWarning("mode 0 Success!");
    }
    public void PublishJoints_1()
    {
        Debug.LogWarning("executed recorded trajectory");
        var request = new TrajectoryServiceRequest();
        var pose_list = new PoseListMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        PoseMsg[] recorded_pose = new PoseMsg[1];

        var pose = new PoseMsg();
        pose.position = new Vector3(0.8f, 0.0f, 0.0f).To<FLU>();
        pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        recorded_pose[0] = pose;
        pose_list.pose_list = recorded_pose;

        request.pose_list = pose_list;

        request.mode = 1;
        Debug.LogWarning("mode 1 sending!");
        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        Debug.LogWarning("mode 1 Success!");
    }

    public void PublishJoints_2()
    {
        Debug.LogWarning("back to detection");
        var request = new TrajectoryServiceRequest();
        var pose_list = new PoseListMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        PoseMsg[] recorded_pose = new PoseMsg[1];

        var pose = new PoseMsg();
        pose.position = new Vector3(0.8f, 0.0f, 0.0f).To<FLU>();
        pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        recorded_pose[0] = pose;
        pose_list.pose_list = recorded_pose;

        request.pose_list = pose_list;

        request.mode = 2;
        Debug.LogWarning("mode 2 sending!");
        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        Debug.LogWarning("mode 2 Success!");
    }
    public void PublishJoints_3()
    {
        Debug.LogWarning("search objects, move to detection pose");
        var request = new TrajectoryServiceRequest();
        var pose_list = new PoseListMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        PoseMsg[] recorded_pose = new PoseMsg[1];

        var pose = new PoseMsg();
        pose.position = new Vector3(0.8f, 0.0f, 0.0f).To<FLU>();
        pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        recorded_pose[0] = pose;
        pose_list.pose_list = recorded_pose;

        request.pose_list = pose_list;

        request.mode = 3;

        Debug.LogWarning("mode 3 sending!");
        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        Debug.LogWarning("mode 3 Success!");
    }

    public void PublishJoints_5()
    {
        Debug.LogWarning("Send Start");
        var request = new TrajectoryServiceRequest();
        request.pose_list = CurrentJointConfig();

        // int  0 represents go to prepare pose (finish and wait next demostration)
        // int  1 represents execute
        // int  2 represents synchronize back (virtual status to real status) 
        // int  3 represents step 1(search objects)
        // int  4 represents simulate include grasp action
        // int  5 represents simulate no grasp
        // int  6 represents play step 2 (grasp part)
        // int  7 represents start new demonstration back to detection mode



        // Pick Pose
        //request.pick_pose = new PoseMsg
        //{
        //    position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),

        //    // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
        //    orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        //};

        //// Place Pose
        //request.place_pose = new PoseMsg
        //{
        //   position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
        //   orientation = m_PickOrientation.To<FLU>()
        //};

        request.mode = 5;
        bool gripper_change = recorder.gripper_change;

        if (gripper_change)
        {
            request.mode = 4;
            Debug.LogWarning("Send mode 4 Success!");
            recorder.gripper_change = false;
        }
        
        

        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        
    }

    public void PublishJoints_6()
    {
        Debug.LogWarning("simulate all trajectories, move to detection pose, then simulate");
        var request = new TrajectoryServiceRequest();
        var pose_list = new PoseListMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        PoseMsg[] recorded_pose = new PoseMsg[1];

        var pose = new PoseMsg();
        pose.position = new Vector3(0.0f, 0.0f, 0.0f).To<FLU>();
        pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        recorded_pose[0] = pose;
        pose_list.pose_list = recorded_pose;

        request.pose_list = pose_list;

        request.mode = 6;

        Debug.LogWarning("mode 6 sending!");
        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        Debug.LogWarning("mode 6 Success!");
    }

    public void PublishJoints_7()
    {
        Debug.LogWarning("simulate all trajectories, move to detection pose, then simulate");
        var request = new TrajectoryServiceRequest();
        var pose_list = new PoseListMsg();
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            pose_list.joints[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        PoseMsg[] recorded_pose = new PoseMsg[1];

        var pose = new PoseMsg();
        pose.position = new Vector3(0.0f, 0.0f, 0.0f).To<FLU>();
        pose.orientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        recorded_pose[0] = pose;
        pose_list.pose_list = recorded_pose;

        request.pose_list = pose_list;

        request.mode = 7;

        Debug.LogWarning("mode 7 sending!");
        m_Ros.SendServiceMessage<TrajectoryServiceResponse>(m_RosServiceName, request, TrajectoryResponse);
        Debug.LogWarning("mode 7 Success!");
    }



    void TrajectoryResponse(TrajectoryServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {

            Debug.LogWarning("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
            Debug.LogWarning("execute succcess!!");

        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///     PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// </summary>
    /// <param name="response"> MoverServiceResponse received from xarm_moveit mover service running in ROS</param>
    /// <returns></returns>
    IEnumerator ExecuteTrajectories(TrajectoryServiceResponse response)
    {
        if (response.trajectories != null)
        {
            Debug.LogWarning(" trajectory length :" + response.trajectories.Length);
            // For every trajectory plan returned
            for (var poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                //Debug.LogWarning(" Pose index :" + poseIndex);
                //Debug.LogWarning(" Points length :" + response.trajectories[poseIndex].joint_trajectory.points);

                foreach (var t in response.trajectories[poseIndex].joint_trajectory.points)
                {
                    var jointPositions = t.positions;
                    var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
                    {

                        var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        m_JointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    //Debug.LogWarning("1" + t.ToString());
                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(k_JointAssignmentWait);

                    //Debug.LogWarning("2" + t.ToString());
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp)
                {
                    CloseGripper();
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(k_PoseAssignmentWait);
            }

            // All trajectories have been executed, open the gripper to place the target cube
            OpenGripper();
        }
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}