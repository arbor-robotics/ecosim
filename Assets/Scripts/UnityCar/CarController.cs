﻿using System;
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
[SerializeField] private float maxSpeed = 2f; // m/s
[SerializeField] private float maxAngularSpeed = 0.7f; // rad/s
[SerializeField] private float motorTorque = 200; // Nm
[SerializeField] private float brakeTorque = 200;  // Nm
[SerializeField] private float forwardSpeedKp = 2.0f;
[SerializeField] private float angularSpeedKp = 20.0f;

public Transform frWheelModel;
public Rigidbody GetRB {
	get { return rigidBody; }
}

private WheelCollider[] wheelColliders;
private Rigidbody rigidBody;
private MotionCommandListener rosListener;

void Awake()
{
	// set the physics clock to 120 Hz
	Time.fixedDeltaTime = 0.008333f;
	// Find the wheel colliders and put them in an array
	// Do not re-order the CD_Colliders in the vehicle model, everything depends on them being RL RR FL FR
	// WC sequence RL RR FL FR
	wheelColliders = gameObject.GetComponentsInChildren<WheelCollider>();
	// Get and configure the vehicle rigidbody
	rigidBody = GetComponent<Rigidbody>();

	rosListener = GetComponent<MotionCommandListener>();
}


void FixedUpdate()
{
	// Debug.Log($"X: {inputX} // Y: {inputY} // R {inputR} // H {inputH}");

	// calculate vehicle velocity in the forward direction
	float currentForwardSpeed = Vector3.Dot(transform.forward, rigidBody.velocity);

	// Recall that we have to negate this to make it right-handed.
	// y is up in Unity.
	float currentAngularSpeed = -rigidBody.angularVelocity.y;

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

	// If we're controlling with the keyboard, listen to that.
	targetForwardSpeed = vInput != 0 ? vInput : rosListener.LinearTwistX;
	targetAngularSpeed = hInput != 0 ? -hInput : rosListener.AngularTwistZ;

	float forwardSpeedError = targetForwardSpeed - currentForwardSpeed;
	float angularSpeedError = targetAngularSpeed - currentAngularSpeed;

	// Debug.Log($"Error: {forwardSpeedError}, {angularSpeedError}");
	rosListener.linearError = forwardSpeedError;
	rosListener.angularError = angularSpeedError;
	rosListener.current = currentForwardSpeed;
	rosListener.target = targetForwardSpeed;

	// Debug.Log($"Fwd err: {forwardSpeedError}, Ang err: {angularSpeedError}");

	// This determines how quickly we drive forward/backward. Linked to linear twist.
	float forwardFactor = forwardSpeedError * forwardSpeedKp;
	float spinFactor = angularSpeedError * angularSpeedKp;

	// This determines how quickly we turn. Linked to

	bool isAccelerating = Mathf.Sign(targetForwardSpeed) == Mathf.Sign(currentForwardSpeed);

	foreach (var wc in wheelColliders)
	{

		// Here we use the naming conventing of the WheelColliders
		// (e.g. "WC_RL" means "WC Rear Left") to determine a WC's
		// side on the robot.
		bool isOnRight = wc.name.Last() == 'R';

		// If our commands are zero, stop the robot.
		if (targetForwardSpeed == 0 && targetAngularSpeed == 0)
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
		else if (Math.Abs(currentForwardSpeed) > maxSpeed && Math.Abs(angularSpeedError) < 0.1) {
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