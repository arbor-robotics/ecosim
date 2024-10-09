using UnityEngine;
// using std_msgs.msg;
using Unity.Collections;
using System.Linq;
using System.Security.Cryptography.X509Certificates;
using System;

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
        // public float degreesPerPixel = 10;
        public float widthPixels = 672;
        public float heightPixels = 376;


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
            for (float i = 0; i < widthPixels; i++)
            {
                for (float j = 0; j < heightPixels; j++)
                {
                    float x = i - (widthPixels / 2); // Range is now (-336 px, +336 px)
                    x /= widthPixels / 2; // Range is now (-1, 1)
                    x *= hFovDegrees / 2;
                    float y = j - (heightPixels / 2);
                    y /= heightPixels / 2; // Range is now (-1, 1)
                    y *= vFovDegrees / 2; // Convert to degrees

                    float ray_x = Mathf.Tan(Mathf.Deg2Rad * x);
                    float ray_y = Mathf.Tan(Mathf.Deg2Rad * y);

                    Vector3 ray_direction = new Vector3(ray_x, ray_y, 1f);

                    RaycastHit hit;
                    // Vector3 rayOrigin = rigidbody.position;

                    if (Physics.Raycast(transform.position, ray_direction, out hit, maxRange))
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