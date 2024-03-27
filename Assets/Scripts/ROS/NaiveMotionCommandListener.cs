using UnityEngine;
using System;
using geometry_msgs.msg;
using sensor_msgs.msg;
using System.Runtime.CompilerServices;

namespace ROS2
{
/// <summary>
/// The main input to this class is Twist messages received by a ROS subscriber.
/// The output is simulated user control commands, with are read by our UnityCar scripts.
/// Recall that UnityCar listens for input through Unity's built-in user input system,
/// which in turn checks for keyboard presses, gamepads, etc. This script basically
/// acts as a virtual gamepad/keyboard, thanks to its ControllerInputX/Y getters.
/// </summary>
public class NaiveMotionCommandListener : MonoBehaviour
{

// Start is called before the first frame update
private ROS2UnityComponent rosUnityComponent;
private ROS2Node rosNode;
private ISubscription<Twist> commandVelSub;
private float current_forward_speed;
private float current_yaw_rate;

private Rigidbody rigidbody;

// Modeled from UserInput.cs
private float controllerInputX;
private float controllerInputY;
private float controllerInputReverse;
private float controllerInputHandBrake;
private float linear_twist_x;
private float angular_twist_z;

// ROS-specific params
public string nodeName;
public string cmdTopic;

// For measuring staleness
private float lastTime = Time.time;
private float currentTime = Time.time;

// Other params
public float speedLimit = 1.0f;
public float stalenessToleranceSeconds = 0.1f;

/// <summary>
/// This is the equivalent to the Left/Right arrow keys.
/// This turns the robot.
/// </summary>
public float ControllerInputX
{
	get
	{
		if (currentTime - lastTime > stalenessToleranceSeconds) {
			return 0f;
		} else {
			return controllerInputX;
		}
	}
}

public float ControllerInputY
{
	get
	{
		if (currentTime - lastTime > stalenessToleranceSeconds) {
			return 0f;
		}
		if (controllerInputY > 0.3f)
			return .5f;
		else if (controllerInputY < -0.3f)
			return -.5f;
		else if (controllerInputY > 0.1f)
			return 0.3f;
		else if (controllerInputY < -0.1f)
			return -.3f;
		else return 0f;
	}
}

public float ControllerInputReverse
{
	get { return 0f; }
}

public float ControllerInputHandBrake
{
	get { return controllerInputHandBrake; }
}

void Start()
{
	rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();
	rigidbody = GetComponent<Rigidbody>();
}

void Update()
{
	if (rosUnityComponent.Ok())
	{
		if (rosNode == null)
		{
			// Set up the node and subscriber.
			rosNode = rosUnityComponent.CreateNode(nodeName);

			commandVelSub = rosNode.CreateSubscription<Twist>(
				"/cmd_vel", commandVelCb);
		}
	}

	// Unity requires us to get our velocity in the main thread-- that's here!
	// Unity "z" is forward-- ROS's "x". Unity "y" is up-- ROS's "z"
	current_forward_speed = GetComponent<Rigidbody>().velocity.magnitude;

	// Negate to make it right-handed
	current_yaw_rate = -1 * GetComponent<Rigidbody>().angularVelocity.y;

	// Unity also requires us to set motor torques in the main thread.
	setControllerInputs();

	currentTime = Time.time;
}

/// <summary>
/// Calculate the current error in linear and angular speed, then apply
/// appropriate simulated user input to minimize the errors.
/// </summary>
void setControllerInputs()
{
	bool isStale = currentTime - lastTime < stalenessToleranceSeconds;

	if (isStale) {
		linear_twist_x = 0f;
		angular_twist_z = 0f;
	}

	// From the docs: https://docs.unity3d.com/ScriptReference/Rigidbody-velocity.html
	// "In most cases you should not modify the velocity directly, as this can result in
	// unrealistic behaviour - use AddForce instead Do not set the velocity of an object
	// every physics step, this will lead to unrealistic physics simulation."

	// Recall that ROS's "x" is Unity's "z," the forward direction.
	rigidbody.velocity = rigidbody.rotation * new UnityEngine.Vector3(0f, 0f, linear_twist_x);

	// Recall that ROS's "z" is Unity's "y," the yaw direction.
	rigidbody.angularVelocity = new UnityEngine.Vector3(0f, -1*angular_twist_z, 0f);

	Debug.Log($"Setting linear x to {linear_twist_x}, yaw rate to {angular_twist_z}");
}

/// <summary>
/// Sets our target speeds from a given Twist message.
/// </summary>
/// <param name="msg">Twist message from subscriber</param>
void commandVelCb(Twist msg)
{
	lastTime = currentTime;
	linear_twist_x = (float)msg.Linear.X;
	angular_twist_z = (float)msg.Angular.Z;
}
}
}  // namespace ROS2
