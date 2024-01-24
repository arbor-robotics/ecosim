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
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.Image> image_pub;

        public string NodeName;
        public string TopicName;
        public uint CameraPixelWidth = 800;
        public uint CameraPixelHeight = 600;

        private new Camera camera;

        void Start()
        {
            ros2Unity = GetComponentInParent<ROS2UnityComponent>();
            camera = GetComponent<Camera>();

            // We want to handle rendering manually.
            // By disabling the camera like this,
            // we prevent Unity from rendering to the screen.
            camera.enabled = false;


            // Flip the axis: ROS assumes an image origin at the top left.
            // In Unity, the origin is bottom left.
            Matrix4x4 scale = Matrix4x4.Scale (new Vector3 (1, -1, 1));

            // camera.targetTexture = renderTexture;
            camera.projectionMatrix *= scale;
        }

        void Update()
        {
            if (ros2Unity.Ok())
            {
                if (ros2Node == null)
                {
                    // Set up the node and publisher.
                    ros2Node = ros2Unity.CreateNode(NodeName);
                    image_pub = ros2Node.CreatePublisher<sensor_msgs.msg.Image>(TopicName);
                }

                Texture2D tex = GetCameraImage();
                
                // Encode the texture in JPG format
                // byte[] bytes = ImageConversion.EncodeToJPG(tex);
                UnityEngine.Object.Destroy(tex);

                // Debug.Log(tex.GetPixelData<byte>(0).Length);

                // Publish to ROS
                sensor_msgs.msg.Image image_msg = new sensor_msgs.msg.Image();

                byte[] image_data = tex.GetPixelData<byte>(0).ToArray();


                // Array.Reverse(image_data);
                image_msg.Data = image_data;
                image_msg.Encoding = "rgba8";
                image_msg.Height = CameraPixelHeight;
                image_msg.Width = CameraPixelWidth;

                image_pub.Publish(image_msg);

                // Write the returned byte array to a file in the project folder
                // File.WriteAllBytes($"/home/main/{NodeName}.jpg", bytes);
            }
        }

        Texture2D FlipTexture(Texture2D original){
            Texture2D flipped = new Texture2D(original.width,original.height);
         
            int xN = original.width;
            int yN = original.height;
         
         
           for(int i=0;i<xN;i++){
              for(int j=0;j<yN;j++){
                  flipped.SetPixel(xN-i-1, j, original.GetPixel(i,j));
              }
           }
            flipped.Apply();
         
            return flipped;
        }

        private Texture2D GetCameraImage()
        {
            // https://discussions.unity.com/t/mirror-flip-camera/56560/2
            // Altering the projection matrix disturbs how normal vectors are calculated
            // during rendering. Use this to reverse the disturbance.
            GL.invertCulling = true;

            int width = (int) CameraPixelWidth;
            int height = (int) CameraPixelHeight;
            Rect rect = new Rect(0, 0, width, height);
            RenderTexture renderTexture = new RenderTexture(width, height, 24);
            Texture2D screenShot = new Texture2D(width, height, TextureFormat.RGBA32, false);

            // By setting targetTexture, we tell Unity to disable rendering to the screen (in theory, at least)
            camera.targetTexture = renderTexture;
            camera.Render();

            RenderTexture.active = renderTexture;
            screenShot.ReadPixels(rect, 0, 0);

            camera.targetTexture = null;
            RenderTexture.active = null;

            Destroy(renderTexture);
            renderTexture = null;

            // Reset this.
            GL.invertCulling = false;

            return screenShot;
        }
    }

}  // namespace ROS2