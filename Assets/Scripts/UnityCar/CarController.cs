using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using ROS2;
using Unity.VisualScripting;
using UnityEngine;

namespace UnityCar
{

public class CarController : MonoBehaviour
{
[SerializeField] private float mass = 1200.0f;
[SerializeField] private Vector3 coG = new Vector3(0.0f, 0.435f, -2.5f);
[SerializeField] private Vector3 inertiaTensor = new Vector3(3600.0f, 3900.0f, 800.0f);
[SerializeField] private float maxSpeed = 2f; // m/s
[SerializeField] private float maxAngularSpeed = 0.7f; // rad/s
[SerializeField] private float motorTorque = 200; // Nm
[SerializeField] private float brakeTorque = 200;  // Nm
[SerializeField] private float forwardSpeedKp = 2.0f;
[SerializeField] private float angularSpeedKp = 20.0f;

public Transform frWheelModel;


public float GetVel {
	get { return vel; }
}
public float GetMass {
	get { return mass; }
}
public Vector3 GetCoG {
	get { return coG; }
}
public Rigidbody GetRB {
	get { return rigidbody; }
}

private WheelCollider[] wheelColliders;
private Rigidbody rigidbody;
private float vel;
private float steerAngle = 0.0f;

// private AeroDynamics aeroDynamics;
private Brakes brakes;
private Engine engine;
private Steering steering;
private Suspension suspension;
private Transmission transmission;
private UserInput userInput;
private MotionCommandListener rosInput;

void Awake()
{
	// aeroDynamics = GetComponent<AeroDynamics>();
	brakes = GetComponent<Brakes>();
	engine = GetComponent<Engine>();
	steering = GetComponent<Steering>();
	suspension = GetComponent<Suspension>();
	transmission = GetComponent<Transmission>();
	userInput = GetComponent<UserInput>();
	rosInput = GetComponent<MotionCommandListener>();


	// set the physics clock to 120 Hz
	Time.fixedDeltaTime = 0.008333f;
	// Find the wheel colliders and put them in an array
	// Do not re-order the CD_Colliders in the vehicle model, everything depends on them being RL RR FL FR
	// WC sequence RL RR FL FR
	wheelColliders = gameObject.GetComponentsInChildren<WheelCollider>();
	// Get and configure the vehicle rigidbody
	rigidbody = GetComponent<Rigidbody>();
	// rB.mass = 800;
	// rB.centerOfMass = coG;
	rigidbody.inertiaTensor = inertiaTensor;
	rigidbody.isKinematic = false;
}


void FixedUpdate()
{
	// Debug.Log($"X: {inputX} // Y: {inputY} // R {inputR} // H {inputH}");

	// calculate vehicle velocity in the forward direction
	vel = transform.InverseTransformDirection(rigidbody.velocity).z;
	float currentForwardSpeed = Vector3.Dot(transform.forward, rigidbody.velocity);

	// Recall that we have to negate this to make it right-handed.
	// y is up in Unity.
	float currentAngularSpeed = -rigidbody.angularVelocity.y;

	// Calculate how close the car is to top speed
	// as a number from zero to one
	float speedFactor = Mathf.InverseLerp(0, maxSpeed, Math.Abs(currentForwardSpeed));

	// Use that to calculate how much torque is available
	// (zero torque at top speed)
	float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);

	float vInput = Input.GetAxis("Vertical");
	float hInput = Input.GetAxis("Horizontal");
	float targetForwardSpeed; // m/s
	float targetAngularSpeed; // rad/s

	if (false)
	{
		// ROS message here!
	} else {
		targetForwardSpeed = vInput * maxSpeed;
		targetAngularSpeed = -hInput * maxAngularSpeed;
	}

	float forwardSpeedError = targetForwardSpeed - currentForwardSpeed;

	float angularSpeedError = targetAngularSpeed - currentAngularSpeed;

	Debug.Log($"Fwd err: {forwardSpeedError}");
	Debug.Log($"Ang err: {angularSpeedError}");

	// This determines how quickly we drive forward/backward. Linked to linear twist.
	float forwardFactor = forwardSpeedError * forwardSpeedKp;
	float spinFactor = angularSpeedError * angularSpeedKp;

	// This determines how quickly we turn. Linked to

	bool isAccelerating = Mathf.Sign(vInput) == Mathf.Sign(currentForwardSpeed);

	foreach (var wc in wheelColliders)
	{

		// Here we use the naming conventing of the WheelColliders
		// (e.g. "WC_RL" means "WC Rear Left") to determine a WC's
		// side on the robot.
		bool isOnRight = wc.name.Last() == 'R';

		// If our commands are zero, stop the robot.
		if (vInput == 0 && hInput == 0)
		{
			// Debug.Log("Stopping!");
			wc.brakeTorque = brakeTorque;
			wc.motorTorque = 0;
		}
		// If the user is trying to go in the opposite direction
		// apply brakes to all wheels
		else if (!isAccelerating && forwardSpeedError > 0.5f)
		{
			// Debug.Log("Braking!");
			wc.brakeTorque = Mathf.Abs(forwardFactor) * brakeTorque;
			wc.motorTorque = 0;
		}
		// If we're exceeding the speed limit, coast
		else if (Math.Abs(currentForwardSpeed) > maxSpeed) {
			Debug.Log("Coasting!");
			wc.motorTorque = 0;
			wc.brakeTorque = 0.1f * brakeTorque;
		}
		// Otherwise, spin the wheels normally.
		else
		{
			wc.motorTorque = forwardFactor * currentMotorTorque;
			wc.brakeTorque = 0;
		}

		// Turning is performed here! Each motor torque is augmented
		// by the spinFactor, which is proportional to angular error
		if (isOnRight) {
			wc.motorTorque += spinFactor * currentMotorTorque;
		} else {
			wc.motorTorque -= spinFactor * currentMotorTorque;
		}
	}
}


void Update()
{
	Transform wheelMeshTransform;

	// update wheel mesh positions to match wheel collider positions
	// slightly convoluted correction for tyre compression which must be corrected locally when WC position update is only available globally

	for (int i = 0; i < 4; i++)
	{
		wheelColliders[i].GetWorldPose(out Vector3 wcPosition, out Quaternion wcRotation);


		wheelMeshTransform = wheelColliders[i].gameObject.transform.GetChild(0);
		wheelMeshTransform.transform.position = wcPosition;
		wheelMeshTransform.transform.rotation = wcRotation;
	}
}

}
}