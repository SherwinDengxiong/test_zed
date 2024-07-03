using System;
using RosMessageTypes.Geometry;
using RosMessageTypes.XarmMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class SourceDestinationPublisher : MonoBehaviour
{
    const int k_NumRobotJoints = 6;

    public static readonly string[] LinkNames =
        { "world/link_base/link1", "/link2", "/link3", "/link4", "/link5", "/link6" };

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/xarm_joints";

    [SerializeField]
    GameObject m_Xarm;
    [SerializeField]
    GameObject m_Target;
    [SerializeField]
    GameObject m_TargetPlacement;
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    UrdfJointRevolute[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<XarmMoveitJointsMsg>(m_TopicName);

        m_JointArticulationBodies = new UrdfJointRevolute[k_NumRobotJoints];

        var linkName = string.Empty;
        //Debug.LogWarning("k_NumRobotJoints " + k_NumRobotJoints);
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_Xarm.transform.Find(linkName).GetComponent<UrdfJointRevolute>();
            //Debug.LogWarning(i + ": " + m_JointArticulationBodies[i]);
        }
    }

    public void Publish()
    {
        var sourceDestinationMessage = new XarmMoveitJointsMsg();
        Debug.LogWarning("send start " + k_NumRobotJoints +" "+ m_JointArticulationBodies.Length);
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            Debug.LogWarning("LOG " + i);
            Debug.LogWarning("LOG "+ i +": " + m_JointArticulationBodies[i].GetPosition());
            sourceDestinationMessage.joints[i] = m_JointArticulationBodies[i].GetPosition();

            Debug.LogWarning("LOG2 " + i + ": " + sourceDestinationMessage.joints[i]);
        }

        Debug.LogWarning("send 1");
        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = m_Target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
        };
        Debug.LogWarning("send 2");

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = m_TargetPlacement.transform.position.To<FLU>(),
            orientation = m_PickOrientation.To<FLU>()
        };

        Debug.LogWarning("send ready");

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, sourceDestinationMessage);
    }
}
