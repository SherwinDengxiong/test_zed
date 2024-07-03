using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Geometry;
using RosMessageTypes.XarmMoveit;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class VehicleController : MonoBehaviour
{
    public string TopicName = "/turtle1/cmd_vel";

    public float head_rotate_index = 1.0f;
    public float vehicle_rotate_index = 1.0f;
    public float vehicle_speed_index = 1.0f;
    public OVRInput.Controller controller;
    public GameObject headObject;


    // ROS Connector
    ROSConnection m_Ros;


    // Start is called before the first frame update
    void Start()
    {
        

        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<VehicleControlMsg>(TopicName);

    }

    // Update is called once per frame
    void Update()
    {
        Debug.LogWarning("updating x and y");

       // prepare_info();
    }
    void prepare_info() {
        var VehicleControl_msg = new VehicleControlMsg();
        

        var vt = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);
        Debug.LogWarning(vt.x+"and, "+vt.y);
        VehicleControl_msg.twist_value.linear.x = vt.y* vehicle_speed_index;
        VehicleControl_msg.twist_value.angular.z = vt.x* vehicle_rotate_index;


        Vector3 head_position = headObject.transform.position;
        Quaternion head_Rotation = headObject.transform.rotation; 

       

        VehicleControl_msg.head_pose.position = head_position.To<FLU>();
        VehicleControl_msg.head_pose.orientation = head_Rotation.To<FLU>();
        Debug.LogWarning(head_Rotation.x+ "and, " + head_Rotation.y+ "and, " + head_Rotation.z+ "and, " + head_Rotation.w);

        m_Ros.Publish(TopicName, VehicleControl_msg);

    }
}
