using UnityEngine;
using System.IO;
using System;
using std_msgs.msg;
using System.Linq;
using System.Threading.Tasks;

namespace ROS2
{
    public class CameraPublisher : MonoBehaviour
    {
        // Start is called before the first frame update
        private ROS2UnityComponent rosUnityComponent;
        private ROS2Node rosNode;
        private IPublisher<sensor_msgs.msg.Image> imagePublisher;
        private IPublisher<sensor_msgs.msg.CameraInfo> cameraInfoPublisher;

        public string nodeName;
        public string cameraName;
        public int cameraPixelWidth = 800;
        public int cameraPixelHeight = 600;

        private new Camera camera;

        void Start()
        {
            rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
            camera = GetComponent<Camera>();

        }

        void Update()
        {
            
        }

    }

}  // namespace ROS2