using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

using std_msgs.msg;


public class PlantingService : MonoBehaviour
{

private ROS2UnityComponent rosUnityComponent;
private ROS2Node rosNode;
private ISubscription<Empty> plantingTriggerSub;
private Publisher<Empty> onSeedlingPlantedPub;

DateTime epochStart;
public string nodeName;
public double plantingDelay = 2.0; // Time it takes to plant a seedling, seconds
private bool seedlingTriggered = false;
private double triggeredTime;


public GameObject[] seedlingPrefabs;

// Start is called before the first frame update
void Start()
{
	rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
	epochStart = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

}

private void plantSeedling()
{
	Debug.Log("Planting!");

	GameObject seedling = Instantiate(seedlingPrefabs[0], transform.position, Quaternion.identity);
	seedling.transform.localScale = new Vector3(0.03f, 0.02f, 0.03f);
}

// Update is called once per frame
void Update()
{
	if (!rosUnityComponent.Ok())
	{
	}
	if (rosNode == null)
	{
		// Set up the node and publisher.
		rosNode = rosUnityComponent.CreateNode(nodeName);
		plantingTriggerSub = rosNode.CreateSubscription<Empty>(
			"/planning/plant_seedling", triggerCb);
		onSeedlingPlantedPub = rosNode.CreatePublisher<Empty>("/events/seedling_planted");
	}
	if (Input.GetKeyDown(KeyCode.Space))
	{
		Debug.Log("Planting!");
		GameObject seedling = Instantiate(seedlingPrefabs[0], transform.position, Quaternion.identity);
		seedling.transform.localScale = new Vector3(0.03f, 0.02f, 0.03f);
	}

	if (seedlingTriggered && (DateTime.UtcNow - epochStart).TotalSeconds - triggeredTime > plantingDelay)
	{
		plantSeedling();
		seedlingTriggered = false;
		onSeedlingPlantedPub.Publish(new Empty());
	}

}
void triggerCb(Empty msg)
{
	triggeredTime = (DateTime.UtcNow - epochStart).TotalSeconds;
	seedlingTriggered = true;
}
}
