using System;
using System.Collections;
using System.Collections.Generic;
using ROS2;
using UnityEngine;

namespace UnityCar
{

public class CarController : MonoBehaviour
{
[SerializeField] private float mass = 1200.0f;
[SerializeField] private Vector3 coG = new Vector3(0.0f, 0.435f, -2.5f);
[SerializeField] private Vector3 inertiaTensor = new Vector3(3600.0f, 3900.0f, 800.0f);
[SerializeField] private float maxSpeed = 2f; // m/s
[SerializeField] private float motorTorque = 200; // Nm
[SerializeField] private float brakeTorque = 200; // Nm

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
	// see if there is a controller script active and if so take input from it
	float inputX = 0.0f;
	float inputY = 0.0f;
	float inputR = 0.0f;
	float inputH = 0.0f;

	if (rosInput.enabled) {
		inputX = rosInput.ControllerInputX;
		inputY = rosInput.ControllerInputY;
		inputR = rosInput.ControllerInputReverse;
		inputH = rosInput.ControllerInputHandBrake;
	}
	if (userInput.enabled && inputX == 0f && inputY == 0f)
	{
		inputX = userInput.ControllerInputX;
		inputY = userInput.ControllerInputY;
		inputR = userInput.ControllerInputReverse;
		inputH = userInput.ControllerInputHandBrake;
	}
	// Debug.Log($"X: {inputX} // Y: {inputY} // R {inputR} // H {inputH}");

	// calculate vehicle velocity in the forward direction
	vel = transform.InverseTransformDirection(rigidbody.velocity).z;
	float forwardSpeed = Vector3.Dot(transform.forward, rigidbody.velocity);

	Debug.Log(forwardSpeed);

	// Calculate how close the car is to top speed
	// as a number from zero to one
	float speedFactor = Mathf.InverseLerp(0, maxSpeed, Math.Abs(forwardSpeed));

	// Use that to calculate how much torque is available
	// (zero torque at top speed)
	float currentMotorTorque = Mathf.Lerp(motorTorque, 0, speedFactor);


	// update steering angle due to input, correct for ackermann and apply steering (if we have a steering script)
	steerAngle = steering.SteerAngle(vel, inputX, steerAngle);
	//wC[2].steerAngle = steering.AckerAdjusted(steerAngle, suspension.GetWheelBase, suspension.GetTrackFront, true);
	//wC[3].steerAngle = steering.AckerAdjusted(steerAngle, suspension.GetWheelBase, suspension.GetTrackFront, false);


	float vInput = Input.GetAxis("Vertical");
	float hInput = Input.GetAxis("Horizontal");
	bool isAccelerating = Mathf.Sign(vInput) == Mathf.Sign(forwardSpeed);

	foreach (var wc in wheelColliders)
	{
		// Apply steering to Wheel colliders that have "Steerable" enabled
		// if (wheel.steerable)
		// {
		// 	wc.steerAngle = hInput * currentSteerRange;
		// }

		Debug.Log(forwardSpeed);
		// If our forward movement command is zero, stop everything.
		if (vInput == 0)
		{
			Debug.Log("Stopping!");
			wc.brakeTorque = brakeTorque;
			wc.motorTorque = 0;
		}
		else if (!isAccelerating)
		{
			Debug.Log("Braking!");
			// If the user is trying to go in the opposite direction
			// apply brakes to all wheels
			wc.brakeTorque = Mathf.Abs(vInput) * brakeTorque;
			wc.motorTorque = 0;
		}
		// If we're exceeding the speed limit, coast
		else if (Math.Abs(forwardSpeed) > maxSpeed) {
			Debug.Log("Coasting!");
			wc.motorTorque = 0;
			wc.brakeTorque = 0.1f * brakeTorque;
		}
		else
		{
			Debug.Log("Spinning!");
			wc.motorTorque = vInput * currentMotorTorque;
			wc.brakeTorque = 0;
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