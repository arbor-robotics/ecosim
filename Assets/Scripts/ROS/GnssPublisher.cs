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
public class GnssPublisher : MonoBehaviour
{
// Start is called before the first frame update
private ROS2UnityComponent rosUnityComponent;
private ROS2Node rosNode;
private IPublisher<PoseWithCovarianceStamped> posePublisher;
// TODO: Transform broadcaster

PoseWithCovarianceStamped currentPose;
NavSatFix currentFix;
UnityEngine.Transform mapOrigin;

public string nodeName;
public string poseTopic;
public string navSatFixTopic;
public string mapFrameId = "map";

void Start()
{
	rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();

	// Get the pose of our map's reference point. In our case,
	// this is a statue. TODO: Parameterize this.
	GameObject referenceObject = GameObject.Find("MapFrameOrigin");
	mapOrigin = referenceObject.transform;
}

void Update()
{
	if (rosUnityComponent.Ok())
	{
		if (rosNode == null)
		{
			// Set up the node and publisher.
			rosNode = rosUnityComponent.CreateNode(nodeName);
			posePublisher = rosNode.CreateSensorPublisher<PoseWithCovarianceStamped>(poseTopic);

			// Messages shouldn't be instantiated before the ROS node is created.
			// https://github.com/RobotecAI/ros2-for-unity/issues/53#issuecomment-1418680445
			currentPose = new PoseWithCovarianceStamped();
		}

		currentPose.Header.Stamp = GetStamp();

		// var diff = transform

		// TODO: Dejank this! WSH
		currentPose.Pose.Pose.Position = new Point
		{
			X = transform.position.x - mapOrigin.position.x, // Left is y in ROS, -x in Unity
			Y = transform.position.z - mapOrigin.position.z, // Forward is x in ROS, z in Unity
			Z = 0f // TODO: Add support for elevation!!
		};

		UnityEngine.Quaternion currentOrientation = transform.rotation.Unity2Ros();

		UnityEngine.Vector3 corrected_eulers = currentOrientation.eulerAngles;
		corrected_eulers[2] += 90f;
		currentOrientation = UnityEngine.Quaternion.Euler(corrected_eulers);

		currentPose.Pose.Pose.Orientation = new geometry_msgs.msg.Quaternion
		{
			W = currentOrientation.w,
			X = currentOrientation.x,
			Y = currentOrientation.y,
			Z = currentOrientation.z
		};

		// Publish everything
		posePublisher.Publish(currentPose);
	}

}

Header GetHeader()
{
	Header header = new Header
	{
		Frame_id = mapFrameId,
		Stamp = GetStamp()
	};

	return header;
}

builtin_interfaces.msg.Time GetStamp()
{
	DateTime epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
	double cur_time = (DateTime.UtcNow - epochStart).TotalSeconds;
	builtin_interfaces.msg.Time stamp = new builtin_interfaces.msg.Time();

	// float currentTime = Time.time;
	int secs = (int)math.floor(cur_time);
	uint nanos = (uint)((cur_time - secs) * 1e9);

	stamp.Sec = secs;
	stamp.Nanosec = nanos;

	return stamp;
}

}

}  // namespace ROS2