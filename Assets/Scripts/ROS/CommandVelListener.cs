using UnityEngine;
using System.IO;
using System;
using std_msgs.msg;
using geometry_msgs.msg;
using sensor_msgs.msg;
using System.Linq;
using System.Threading.Tasks;
using Unity.Collections;
using System.Security.Cryptography;
using Unity.Mathematics;
using Unity.VisualScripting;

namespace ROS2
{
    public class CommandVelListener : MonoBehaviour
    {

        private WheelCollider[] wC;

        // Start is called before the first frame update
        private ROS2UnityComponent rosUnityComponent;
        private ROS2Node rosNode;
        private ISubscription<TwistStamped> commandVelSub;
        

        // TODO: Transform broadcaster

        PoseWithCovarianceStamped currentPose;
        NavSatFix currentFix;
        UnityEngine.Transform mapOrigin;

        public string nodeName;
        public string cmdTopic;

        void Start()
        {
            rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();

            // Find all WheelCollider objects as children of this object.
            wC = gameObject.GetComponentsInChildren<WheelCollider>();
        }

        void Update()
        {
            if (rosUnityComponent.Ok())
            {
                if (rosNode == null)
                {
                    // Set up the node and subscriber.
                    rosNode = rosUnityComponent.CreateNode(nodeName);
                    
                    commandVelSub =  rosNode.CreateSubscription<TwistStamped>(
                                    "/cmd_vel", commandVelCB);
                }   
            }
        }

       

        builtin_interfaces.msg.Time GetStamp()
        {
            builtin_interfaces.msg.Time stamp = new builtin_interfaces.msg.Time();

            float currentTime = Time.time;
            int secs = (int)math.floor(currentTime);
            uint nanos = (uint)((currentTime - secs) * 1e9);

            stamp.Sec = secs;
            stamp.Nanosec = nanos;

            return stamp;
        }

        void commandVelCB(TwistStamped msg){
            Debug.Log(msg.Twist.Linear.X); // m/s
            Debug.Log(msg.Twist.Angular.Z); // rad/s
            Debug.Log("Subscriber message");

            // wC[0].brakeTorque = something
            // wC[0].engineTorque = something
            // wC[0].steerAngle = something
        }
    }
}  // namespace ROS2