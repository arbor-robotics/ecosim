using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using JetBrains.Rider.Unity.Editor;
using UnityEngine;

public class ArcadeCarController : MonoBehaviour
{

    [SerializeField] bool showDebug;
    [SerializeField] GameObject[] rayPoints;
    [SerializeField] GameObject[] wheels;
    [SerializeField] float wheelRadius;
    [SerializeField] float suspensionDistance;
    [SerializeField] float springFactor; // 0 to 1
    [SerializeField] float springDamper;
    [SerializeField] float sideGridFactor; // 0 to 1. How much the tire resists slip in the side direction.
    [SerializeField] float forwardGripFactor; // 0 to 1. How much the tire resists slip in the forward direction.
    [SerializeField] float tireMass;
    [SerializeField] float mass;
    [SerializeField] float throttleMultiplier;
    [SerializeField] float turnMultiplier;
    [SerializeField] float speedLimit; // m/s
    [SerializeField] float forceLimit; // N



    float springStrength;

    private List<Vector3> netForces;

    new Rigidbody rigidbody;

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
        rigidbody.mass = mass;

        springStrength = mass * 9.8f / 4.0f / (suspensionDistance / 2);

        netForces = new() { new Vector3(), new Vector3(), new Vector3(), new Vector3() };
    }

    void FixedUpdate()
    {
        ResetNetForces();
        // VisualizeWheelAxes();
        // VisualizeRaycasts();
        AddSuspensionForces();
        AddSteeringForces();
        AddThrottle();
        ApplyNetForces();
        VisualizeNetForces();
    }

    void ResetNetForces()
    {
        netForces = new() { new Vector3(), new Vector3(), new Vector3(), new Vector3() };
    }


    void ApplyNetForces()
    {
        for (int i = 0; i < netForces.Count; i++)
        {
            Vector3 force = netForces[i];

            float magnitude = force.magnitude;


            // Apply force limits
            force /= magnitude;
            magnitude = Math.Min(forceLimit, magnitude);
            magnitude = Math.Max(-forceLimit, magnitude);
            force *= magnitude;

            if (float.IsNaN(force.x))
                continue;

            GameObject wheel = rayPoints[i];

            rigidbody.AddForceAtPosition(force, wheel.transform.position);

        }
    }

    void VisualizeNetForces(float scale = 0.002f)
    {
        for (int i = 0; i < rayPoints.Length; i++)
        {
            GameObject wheel = rayPoints[i];
            Vector3 force = netForces[i];

            if (showDebug)
                Debug.DrawLine(wheel.transform.position, wheel.transform.position + force * scale, Color.yellow);
        }
    }

    void VisualizeWheelAxes()
    {
        foreach (GameObject wheel in rayPoints)
        {
            Vector3 start = wheel.transform.position;

            // x axis in red
            Vector3 end = wheel.transform.position;
            end.x += 1;
            Debug.DrawLine(start, end, Color.red);

            // y axis in green
            end = wheel.transform.position;
            end.y += 1;
            Debug.DrawLine(start, end, Color.green);

            // z axis in blue
            end = wheel.transform.position;
            end.z += 1;
            Debug.DrawLine(start, end, Color.blue);
        }
    }

    private void AddSuspensionForces()
    {
        for (int i = 0; i < rayPoints.Length; i++)
        {

            GameObject rayOrigin = rayPoints[i];
            GameObject wheel = wheels[i];

            // Shoot a ray down

            RaycastHit hit;

            float wheelRestDistance = wheelRadius + suspensionDistance;

            if (Physics.Raycast(rayOrigin.transform.position, -Vector3.up, out hit, wheelRestDistance))
            {
                float wheelOffset = hit.distance - wheelRestDistance;

                if (showDebug)
                {
                    Debug.Log($"Ground is {hit.distance} meters below {rayOrigin.name}. Wheel pushed up {wheelOffset}");
                    Debug.DrawLine(rayOrigin.transform.position, hit.point, Color.magenta);
                }

                Vector3 wheelWorldVel = rigidbody.GetPointVelocity(rayOrigin.transform.position);

                float offset = wheelRestDistance - hit.distance;
                float vel = Vector3.Dot(rayOrigin.transform.up, wheelWorldVel);

                float force = (offset * springStrength) - (vel * springDamper);

                netForces[i] += rayOrigin.transform.up * force;

                Vector3 wheelPosition = hit.point;
                wheelPosition.y += wheelRadius;
                wheel.transform.position = wheelPosition;
            }
        }
    }

    private void AddSteeringForces()
    {
        for (int i = 0; i < rayPoints.Length; i++)
        {
            GameObject wheel = rayPoints[i];

            Vector3 wheelWorldVel = rigidbody.GetPointVelocity(wheel.transform.position);

            // Direction we don't want to wheel to roll in.
            Vector3 slipDirection = wheel.transform.right;

            float slipVel = Vector3.Dot(slipDirection, wheelWorldVel);

            float desiredVelChange = -slipVel * sideGridFactor;

            float desiredAcceleration = desiredVelChange / Time.fixedDeltaTime;

            netForces[i] += slipDirection * tireMass * desiredAcceleration;
        }
    }

    private void AddThrottle()
    {
        float throttle = Input.GetAxis("Vertical");
        float turn = Input.GetAxis("Horizontal");

        float carSpeed = Vector3.Dot(transform.forward, rigidbody.velocity);

        if (carSpeed > speedLimit && throttle >= 0f)
        {
            if (showDebug)
                Debug.Log("Car at speed limit. Coasting.");
        }
        else
        {
            for (int i = 0; i < rayPoints.Length; i++)
            {
                GameObject wheel = rayPoints[i];

                Vector3 accelDir = wheel.transform.forward;
                Vector3 wheelWorldVel = rigidbody.GetPointVelocity(wheel.transform.position);

                float wheelForwardSpeed = Vector3.Dot(accelDir, wheelWorldVel);

                // Apply brakes
                if (Math.Abs(throttle) < 0.1f && Math.Abs(turn) < 0.1)
                {
                    float desiredVelChange;
                    if (rigidbody.velocity.magnitude > .1)
                    {
                        forwardGripFactor -= 0.1f;
                    }
                    else
                    {
                        forwardGripFactor += 0.1f;
                    }
                    forwardGripFactor = Math.Min(0f, forwardGripFactor);
                    forwardGripFactor = Math.Max(5f, forwardGripFactor);
                    desiredVelChange = -wheelForwardSpeed * forwardGripFactor;
                    float desiredSlipAcceleration = desiredVelChange / Time.fixedDeltaTime;
                    netForces[i] += accelDir * tireMass * desiredSlipAcceleration;
                }



                netForces[i] += accelDir * throttle * throttleMultiplier;

            }
        }

        rigidbody.AddTorque(transform.up * turn * turnMultiplier);
    }

    // Update is called once per frame
    void Update()
    {

    }
}
