using UnityEngine;
using System.IO;
using System;
using std_msgs.msg;
using System.Linq;
using System.Threading.Tasks;
using Unity.Collections;
using System.Security.Cryptography;

namespace ROS2
{
    public class GnssPublisher : MonoBehaviour
    {
        // Start is called before the first frame update
        private ROS2UnityComponent rosUnityComponent;
        private ROS2Node rosNode;
        private IPublisher<geometry_msgs.msg.PoseWithCovarianceStamped> posePublisher;
        private IPublisher<sensor_msgs.msg.NavSatFix> navSatFixPublisher;
        // TODO: Transform broadcaster

        public string nodeName;
        public string poseTopic;
        public string navSatFixTopic;

        void Start()
        {
            rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
        }

        void Update()
        {
            if (rosUnityComponent.Ok())
            {
                if (rosNode == null)
                {
                    // Set up the node and publisher.
                    rosNode = rosUnityComponent.CreateNode(nodeName);
                    posePublisher = rosNode.CreateSensorPublisher<geometry_msgs.msg.PoseWithCovarianceStamped>(poseTopic);
                    navSatFixPublisher = rosNode.CreateSensorPublisher<sensor_msgs.msg.NavSatFix>(navSatFixTopic);
                }
                
            }
            
        }

    }

}  // namespace ROS2