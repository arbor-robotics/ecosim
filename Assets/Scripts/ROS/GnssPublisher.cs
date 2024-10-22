using UnityEngine;
using System.IO;
using System;
// using std_msgs.msg;
// using geometry_msgs.msg;
// using sensor_msgs.msg;
using System.Linq;
using System.Threading.Tasks;
using Unity.Collections;
using System.Security.Cryptography;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEngine.Assertions;


namespace ROS2
{
	public class GnssPublisher : MonoBehaviour
	{
		// Start is called before the first frame update
		// private ROS2UnityComponent rosUnityComponent;
		// private ROS2Node rosNode;
		// private IPublisher<PoseWithCovarianceStamped> posePublisher;
		// private IPublisher<NavSatFix> navSatFixPublisher;
		// TODO: Transform broadcaster

		// PoseWithCovarianceStamped currentPose;
		// NavSatFix currentFix;
		UnityEngine.Transform mapOrigin;


		public float maxNoiseCm = 10f;
		WebsocketBridge websocketBridge;

		// YES this is hardcoded.
		// NO I don't have time to change it.
		// WSH Oct 22 '24
		float topLeftLat = 40.44823504f;
		float topLeftLon = -79.95278458f;
		float topLeftEastM = 0f;
		float topLeftNorthM = 1177f;
		float botRightLat = 40.43743552f;
		float botRightLon = -79.93809829f;
		float botRightEastM = 1177f;
		float botRightNorthM = 0f;


		void Start()
		{
			websocketBridge = GetComponent<WebsocketBridge>();
		}

		void Update()
		{
			// 1. Get ego latitude and longitude as floats
			float egoEastM = transform.position.x;
			float egoNorthtM = transform.position.z;
			float egoAltM = transform.position.y;

			float latDelta = topLeftLat - botRightLat;
			float lonDelta = botRightLon - topLeftLon; // Positive, ~0.01469
			float eastMDelta = botRightEastM - topLeftEastM;
			float northMDelta = topLeftNorthM - botRightNorthM;

			float eastFraction = (egoEastM - topLeftEastM) / eastMDelta;
			float northFraction = (egoNorthtM - botRightNorthM) / northMDelta;

			// Debug.Log($"{eastFraction}, {northFraction}");

			float egoLon = topLeftLon + lonDelta * eastFraction;
			float egoLat = botRightLat + latDelta * northFraction;

			// 2. Convert to bytes and store in an array

			byte[] fixBytes = new byte[17]; // first byte is dtype, plus 4 bytes per float, (lat, lon, alt, heading)

			int byte_idx = 0;

			// Debug.Log($"{egoLon}, {egoLat}");

			byte[] latBytes = BitConverter.GetBytes(egoLat);
			byte[] lonBytes = BitConverter.GetBytes(egoLon);
			byte[] altBytes = BitConverter.GetBytes(egoAltM);

			latBytes.CopyTo(fixBytes, 1);
			lonBytes.CopyTo(fixBytes, 5);
			altBytes.CopyTo(fixBytes, 9);

			fixBytes[0] = (byte)KISS.MessageType.GNSS_FIX;

			// Debug.Log($"{lonBytes.Length}, {lonBytes}");

			string bytes_as_string = "";

			foreach (byte b in fixBytes)
			{
				bytes_as_string += $"{b}_";
			}

			Debug.Log($"{bytes_as_string}");

			// byte[] kiss_msg = new byte[] { (byte)KISS.MessageType.GNSS_FIX }.Concat(poseBytes).ToArray();
			websocketBridge.SendBytes(fixBytes);
		}
	}

}  // namespace ROS2