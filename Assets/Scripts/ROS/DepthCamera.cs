using UnityEngine;
// using std_msgs.msg;
using Unity.Collections;
using System.Linq;

namespace ROS2
{
    public class DepthCamera : MonoBehaviour
    {
        // Start is called before the first frame update
        // private ROS2UnityComponent rosUnityComponent;
        // private ROS2Node rosNode;
        // private IPublisher<sensor_msgs.msg.Image> imagePublisher;
        // private IPublisher<sensor_msgs.msg.CameraInfo> cameraInfoPublisher;

        public string nodeName;
        public string cameraName;
        public float hFovDegrees = 101;
        public float vFovDegrees = 68;
        public float degreesPerPixel = 10;


        public new Camera camera;

        WebsocketBridge websocketBridge;

        Rect rect;

        Rigidbody rigidbody;

        float maxRange = 10f; // meters

        bool showDebug = true;

        // Publisher imagePub;

        void Start()
        {
            websocketBridge = GetComponent<WebsocketBridge>();
            rigidbody = GetComponent<Rigidbody>();
        }

        void Update()
        {
            for (float i = 0; i < hFovDegrees; i += degreesPerPixel)
            {
                for (float j = 0; j < vFovDegrees; j += degreesPerPixel)
                {
                    RaycastHit hit;
                    // Vector3 rayOrigin = rigidbody.position;

                    if (Physics.Raycast(transform.position, Vector3.forward, out hit, maxRange))
                    {
                        float wheelOffset = hit.distance - maxRange;

                        if (showDebug)
                        {
                            // Debug.Log($"Ground is {hit.distance} meters below self. Wheel pushed up {wheelOffset}");
                            Debug.DrawLine(transform.position, hit.point, Color.magenta);
                        }
                    }
                }
            }

        }

    }

}  // namespace ROS2