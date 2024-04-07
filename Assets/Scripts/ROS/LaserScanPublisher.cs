using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using sensor_msgs.msg;
using Unity.VisualScripting;
using UnityEngine;
using Unity.Mathematics;


namespace ROS2
{
public class LaserScanPublisher : MonoBehaviour
{
private ROS2UnityComponent rosUnityComponent;
private ROS2Node rosNode;
private IPublisher<sensor_msgs.msg.LaserScan> scanPublisher;

[SerializeField] private string nodeName = "arborsim_laser_scanner";
[SerializeField] private string scanTopic = "/scan";
[SerializeField] private float angleMin = -Mathf.PI / 3;
[SerializeField] private float angleMax = Mathf.PI / 3;
[SerializeField] private float angleIncrement = Mathf.Deg2Rad * 10;
[SerializeField] private float rangeMin = 0.1f; // meters
[SerializeField] private float rangeMax = 10.0f; // meters

private LaserScan scanMsg;


// Start is called before the first frame update
void Start()
{
	rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();


}

// Update is called once per frame
void Update()
{
	if (rosNode == null)
	{

		// Set up the node and publisher.
		rosNode = rosUnityComponent.CreateNode(nodeName);
		scanPublisher = rosNode.CreatePublisher<sensor_msgs.msg.LaserScan>(scanTopic);

		scanMsg = new LaserScan();
		scanMsg.Angle_min = angleMin;
		scanMsg.Angle_max = angleMax;
		scanMsg.Angle_increment = angleIncrement;
		scanMsg.Range_min = rangeMin;
		scanMsg.Range_max = rangeMax;
		scanMsg.Scan_time = 1f/60; // What is this actually?
		scanMsg.Header.Frame_id = "base_link"; // TODO: Make this the sensor frame. WSH
	}
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

void FixedUpdate()
{
	if (rosNode == null) {
		return;
	}

	RaycastHit hit;

	float yaw = transform.rotation.eulerAngles.y;
	float pitch = transform.rotation.eulerAngles.x;

	// float theta = 30;

	List<float> ranges = new List<float>();

	for (float theta = angleMin; theta <= angleMax; theta += angleIncrement)
	{
		Vector3 rayDirection = Quaternion.Euler(pitch, -theta * Mathf.Rad2Deg, 0) * transform.TransformDirection(Vector3.forward);

		if (Physics.Raycast(transform.position, rayDirection, out hit, 10))
		{
			Debug.DrawRay(transform.position, rayDirection * hit.distance, Color.red);
			ranges.Add(hit.distance);
		} else {
			ranges.Add(10f);
		}
	}
	// intensities.Add(1f);

	scanMsg.Header.Stamp = GetStamp();
	scanMsg.Ranges = ranges.ToArray();
	scanPublisher.Publish(scanMsg);
}

}
}