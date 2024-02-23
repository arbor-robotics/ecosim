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

            // Get the pose of our map's reference point. In our case,
            // this is a statue. TODO: Parameterize this.
            GameObject referenceObject = GameObject.FindGameObjectWithTag("MapFrameOrigin");

            if (!referenceObject) {
                Debug.LogError("Could not locate map origin. Is your reference point tagged with MapFrameOrigin?");
            }

            mapOrigin = referenceObject.transform;
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
                    


                    // Messages shouldn't be instantiated before the ROS node is created.
                    // https://github.com/RobotecAI/ros2-for-unity/issues/53#issuecomment-1418680445
                    currentPose = new PoseWithCovarianceStamped();
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
            Debug.Log("HelloWorld");
        }

    }

    

}  // namespace ROS2