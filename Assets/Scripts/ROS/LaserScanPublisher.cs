using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace ROS2
{
public class LaserScanPublisher : MonoBehaviour
{
private ROS2UnityComponent rosUnityComponent;
private ROS2Node rosNode;
private IPublisher<sensor_msgs.msg.LaserScan> scanPublisher;

[SerializeField] private string nodeName = "arborsim_laser_scanner";
[SerializeField] private string scanTopic = "/scan";
[SerializeField] private float angleMin = -Mathf.PI / 2;
[SerializeField] private float angleMax = Mathf.PI / 2;
[SerializeField] private float angleIncrement = Mathf.Deg2Rad * 1;
[SerializeField] private float rangeMin = 0.1f; // meters
[SerializeField] private float rangeMax = 10.0f; // meters


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
	}
}

void FixedUpdate()
{
	Vector3 fwd = transform.TransformDirection(Vector3.forward);

	if (Physics.Raycast(transform.position, fwd, rangeMax))
		Debug.Log("There is something in front of the object!");
}

}
}