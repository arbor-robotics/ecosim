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
public class MotionCommandListener : MonoBehaviour
{

private WheelCollider wheel_rear_left, wheel_rear_right, wheel_front_left, wheel_front_right;
private float torque_rl, torque_rr, torque_fl, torque_fr;

// Start is called before the first frame update
private ROS2UnityComponent rosUnityComponent;
private ROS2Node rosNode;
private ISubscription<Twist> commandVelSub;
private float current_forward_speed;
private float current_yaw_rate;

// Modeled from UserInput.cs
private float controllerInputX;
private float controllerInputY;
private float controllerInputReverse;
private float controllerInputHandBrake;
private float linear_twist_x;
private float angular_twist_z;

public float ControllerInputX
{
	get {
		return controllerInputX;
	}
}

public float ControllerInputY
{
	get
	{
		return controllerInputY;
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


PoseWithCovarianceStamped currentPose;
NavSatFix currentFix;
UnityEngine.Transform mapOrigin;

public string nodeName;
public string cmdTopic;

void Start()
{
	rosUnityComponent = GetComponentInParent<ROS2UnityComponent>();

	// Find all WheelCollider objects as children of this object.
	wheel_rear_left = transform.Find("WC_RL").gameObject.GetComponent<WheelCollider>();
	wheel_rear_right = transform.Find("WC_RR").gameObject.GetComponent<WheelCollider>();
	wheel_front_left = transform.Find("WC_FL").gameObject.GetComponent<WheelCollider>();
	wheel_front_right = transform.Find("WC_FR").gameObject.GetComponent<WheelCollider>();
	// rigidbody = gameObject.GetComponent<Rigidbody>();
}

void Update()
{
	if (rosUnityComponent.Ok())
	{
		if (rosNode == null)
		{
			// Set up the node and subscriber.
			rosNode = rosUnityComponent.CreateNode(nodeName);

			commandVelSub =  rosNode.CreateSubscription<Twist>(
				"/cmd_vel", commandVelCb);
		}
	}

	// Unity requires us to get our velocity in the main thread-- that's here!
	// Unity "z" is forward-- ROS's "x". Unity "y" is up-- ROS's "z"
	current_forward_speed = GetComponent<Rigidbody>().velocity.z;

	// Negate to make it right-handed
	current_yaw_rate = -1*GetComponent<Rigidbody>().angularVelocity.y;

	// Unity also requires us to set motor torques in the main thread.
	setControllerInputs();
}

void setControllerInputs()
{
	float linear_speed_error = linear_twist_x - current_forward_speed;
	float yaw_rate_error = angular_twist_z - current_yaw_rate;
	float Kp_linear = 4.0f;
	float Kp_angular = 5.0f;
	float max_input_x = 5.0f;

	controllerInputY = Math.Max(Math.Min(Kp_linear * linear_speed_error, 1.0f), -1f);
	controllerInputX = Math.Max(
		Math.Min(
			Kp_angular * yaw_rate_error,
			max_input_x), -max_input_x);
	Debug.Log($"Lin. E: {linear_speed_error}, Ang: {yaw_rate_error}");
}

void commandVelCb(Twist msg)
{
	linear_twist_x = (float) msg.Linear.X;
	angular_twist_z = (float) msg.Angular.Z;
}
}
}  // namespace ROS2