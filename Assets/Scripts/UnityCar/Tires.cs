using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UnityCar
{

public class Tires : MonoBehaviour
{

[SerializeField] private float forwardExtremumSlip = 0.6f;
[SerializeField] private float forwardExtremumValue = 3.0f;
[SerializeField] private float forwardAsymptoteSlip = 0.7f;
[SerializeField] private float forwardAsymptoteValue = 2.0f;
[SerializeField] private float forwardStiffness = 1.1f;

[SerializeField] private float sidewaysExtremumSlip = 1f;
[SerializeField] private float sidewaysExtremumValue = 0.7f;
[SerializeField] private float sidewaysAsymptoteSlip = 0.8f;
[SerializeField] private float sidewaysAsymptoteValue = 2.0f;
[SerializeField] private float sidewaysStiffness = 1.1f;

private readonly WheelFrictionCurve[] forwardWheelFrictionCurves = new WheelFrictionCurve[4];
private readonly WheelFrictionCurve[] sidewaysWheelFrictionCurves = new WheelFrictionCurve[4];

public WheelFrictionCurve[] GetWFCLong {
	get { return forwardWheelFrictionCurves; }
}
public WheelFrictionCurve[] GetWFCLat {
	get { return sidewaysWheelFrictionCurves; }
}

void Start()
{

	// get wheelcolliders and particles
	WheelCollider[] wC = gameObject.GetComponentsInChildren<WheelCollider>();

	// WFC characteristics rear tyres
	for (int i = 0; i < 2; i++)
	{
		// forward slip (% of forward travel) versus normalised load
		forwardWheelFrictionCurves[i].extremumSlip = forwardExtremumSlip;
		forwardWheelFrictionCurves[i].extremumValue = forwardExtremumValue;
		forwardWheelFrictionCurves[i].asymptoteSlip = forwardAsymptoteSlip;
		forwardWheelFrictionCurves[i].asymptoteValue = forwardAsymptoteValue;
		forwardWheelFrictionCurves[i].stiffness = forwardStiffness;
		// sideways slip = radians slip versus normalised load
		sidewaysWheelFrictionCurves[i].extremumSlip = sidewaysExtremumSlip;
		sidewaysWheelFrictionCurves[i].extremumValue = sidewaysExtremumValue;
		sidewaysWheelFrictionCurves[i].asymptoteSlip = sidewaysAsymptoteSlip;
		sidewaysWheelFrictionCurves[i].asymptoteValue = sidewaysAsymptoteValue;
		sidewaysWheelFrictionCurves[i].stiffness = sidewaysStiffness;
	}

	// WFC characteristics front tires
	// We'll use the same params for rear and front
	for (int i = 2; i < 4; i++)
	{
		// forward slip (% of forward travel) versus normalised load
		forwardWheelFrictionCurves[i].extremumSlip = forwardExtremumSlip;
		forwardWheelFrictionCurves[i].extremumValue = forwardExtremumValue;
		forwardWheelFrictionCurves[i].asymptoteSlip = forwardAsymptoteSlip;
		forwardWheelFrictionCurves[i].asymptoteValue = forwardAsymptoteValue;
		forwardWheelFrictionCurves[i].stiffness = forwardStiffness;
		// sideways slip = radians slip versus normalised load
		sidewaysWheelFrictionCurves[i].extremumSlip = sidewaysExtremumSlip;
		sidewaysWheelFrictionCurves[i].extremumValue = sidewaysExtremumValue;
		sidewaysWheelFrictionCurves[i].asymptoteSlip = sidewaysAsymptoteSlip;
		sidewaysWheelFrictionCurves[i].asymptoteValue = sidewaysAsymptoteValue;
		sidewaysWheelFrictionCurves[i].stiffness = sidewaysStiffness;
	}

	// Assign the WFC data to the wheel colliders
	// WC sequence RL RR FL FR

	for (int i = 0; i < 4; i++)
	{
		wC[i].ConfigureVehicleSubsteps(30, 8, 20);
		wC[i].forwardFriction = forwardWheelFrictionCurves[i];
		wC[i].sidewaysFriction = sidewaysWheelFrictionCurves[i];
	}

}

}
}