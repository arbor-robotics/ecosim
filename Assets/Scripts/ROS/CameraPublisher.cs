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
        private RenderTexture renderTexture;
        Texture2D screenShot;
        byte[] rawCameraData;

        private new Camera camera;

        void Start()
        {
            rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
            camera = GetComponent<Camera>();
            renderTexture = new RenderTexture(cameraPixelWidth, cameraPixelHeight, 24);

            // 4 bytes per pixel (RGBA)
            rawCameraData = new byte[cameraPixelWidth * cameraPixelHeight * 4];

            screenShot = new Texture2D(cameraPixelWidth, cameraPixelHeight, TextureFormat.RGBA32, false);
            camera.targetTexture = renderTexture;
        }

        /// <summary>
        /// SLOW conversion from NativeArray to byte[].
        /// Likely slow due to GPU memory access, but that's just a theory.
        /// </summary>
        /// <param name="from">NativeArray with pixel data</param>
        void PopulateBytesFromNativeArray(NativeArray<byte> from)
        {
            // var prev = Time.time;
            for (int i = 0; i < from.Length; i++)
            {
                rawCameraData[i] = from[i];
            }
            // var elapsed = Time.time - prev;
            // Debug.Log("Took "+elapsed);
        }

        void Update()
        {

            if (rosUnityComponent.Ok())
            {
                if (rosNode == null)
                {
                    // Set up the node and publisher.
                    rosNode = rosUnityComponent.CreateNode(nodeName);
                    imagePublisher = rosNode.CreatePublisher<sensor_msgs.msg.Image>("/camera/" + cameraName + "/image_color");
                    cameraInfoPublisher = rosNode.CreatePublisher<sensor_msgs.msg.CameraInfo>("/camera/" + cameraName + "/camera_info");
                }
                sensor_msgs.msg.Image image_msg = new sensor_msgs.msg.Image();

                image_msg.Data = rawCameraData;
                image_msg.Encoding = "rgba8";
                image_msg.Height = (uint)cameraPixelHeight;
                image_msg.Width = (uint)cameraPixelWidth;
                imagePublisher.Publish(image_msg);
            }
            camera.Render();

            Rect rect = new Rect(0, 0, cameraPixelWidth, cameraPixelHeight);

            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(rect, 0, 0);
            RenderTexture.active = null;

            // Graphics.CopyTexture(renderTexture, screenShot);

            // byte[] imageData = screenShot.EncodeToPNG();
            var pixels = screenShot.GetPixelData<byte>(0);
            // Debug.Log(pixels.Length);
            PopulateBytesFromNativeArray(pixels);
            // byte[] imageData = screenShot.GetRawTextureData<byte>().ToArray();
            // pixels = null;
        }

    }

}  // namespace ROS2