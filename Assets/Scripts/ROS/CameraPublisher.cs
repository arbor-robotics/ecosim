using UnityEngine;
using System.IO;
using System;
using std_msgs.msg;

namespace ROS2
{
    public class CameraPublisher : MonoBehaviour
    {
        // Start is called before the first frame update
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.Image> image_pub;

        private RenderTexture renderTexture;
        private Texture2D texture2D;

        public string NodeName;
        public string TopicName;
        public int CameraPixelWidth = 800;
        public int CameraPixelHeight = 600;

        private new Camera camera;

        void Start()
        {
            ros2Unity = GetComponentInParent<ROS2UnityComponent>();
            camera = GetComponent<Camera>();

            renderTexture = new RenderTexture(84, 84, 24);
            texture2D = new Texture2D(84, 84, TextureFormat.RGB24, false);
            camera.targetTexture = renderTexture;
        }

        void Update()
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    ros2Node = ros2Unity.CreateNode(NodeName);
                    image_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(TopicName);
                }

                Texture2D tex = GetCameraImage();

                // Encode the texture in JPG format
                byte[] bytes = ImageConversion.EncodeToJPG(tex);
                UnityEngine.Object.Destroy(tex);

                Debug.Log(tex.GetPixelData<Color32>(0).Length);

                // Publish to ROS


                // Write the returned byte array to a file in the project folder
                File.WriteAllBytes($"/home/main/{NodeName}.jpg", bytes);
            }
        }

        private Texture2D GetCameraImage()
        {
            int width = 800;
            int height = 600;
            Rect rect = new Rect(0, 0, width, height);
            RenderTexture renderTexture = new RenderTexture(width, height, 24);
            Texture2D screenShot = new Texture2D(width, height, TextureFormat.RGBA32, false);

            // By setting targetTexture, we tell Unity to disable rendering to the screen.
            camera.targetTexture = renderTexture;
            camera.Render();

            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(rect, 0, 0);

            camera.targetTexture = null;
            RenderTexture.active = null;

            Destroy(renderTexture);
            renderTexture = null;
            return screenShot;
        }
    }

}  // namespace ROS2