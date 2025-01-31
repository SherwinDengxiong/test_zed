//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.XarmGripper
{
    [Serializable]
    public class MoveGoal : Message
    {
        public const string k_RosMessageName = "xarm_gripper/Move";
        public override string RosMessageName => k_RosMessageName;

        public float target_pulse;
        public float pulse_speed;

        public MoveGoal()
        {
            this.target_pulse = 0.0f;
            this.pulse_speed = 0.0f;
        }

        public MoveGoal(float target_pulse, float pulse_speed)
        {
            this.target_pulse = target_pulse;
            this.pulse_speed = pulse_speed;
        }

        public static MoveGoal Deserialize(MessageDeserializer deserializer) => new MoveGoal(deserializer);

        private MoveGoal(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.target_pulse);
            deserializer.Read(out this.pulse_speed);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.target_pulse);
            serializer.Write(this.pulse_speed);
        }

        public override string ToString()
        {
            return "MoveGoal: " +
            "\ntarget_pulse: " + target_pulse.ToString() +
            "\npulse_speed: " + pulse_speed.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Goal);
        }
    }
}
