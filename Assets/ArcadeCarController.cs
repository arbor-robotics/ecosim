using System.Collections;
using System.Collections.Generic;
using JetBrains.Rider.Unity.Editor;
using UnityEngine;

public class ArcadeCarController : MonoBehaviour
{

    [SerializeField] GameObject[] rayPoints;
    [SerializeField] float wheelRadius;
    [SerializeField] float suspensionDistance;
    [SerializeField] float springFactor; // 0 to 1
    [SerializeField] float springDamper;
    [SerializeField] float mass;
    float springStrength;

    new Rigidbody rigidbody;

    // Start is called before the first frame update
    void Start()
    {
        rigidbody = GetComponent<Rigidbody>();
        rigidbody.mass = mass;

        springStrength = mass * 9.8f / 4.0f / (suspensionDistance / 2);
    }

    void FixedUpdate()
    {
        VisualizeWheelAxes();
        // VisualizeRaycasts();
        Suspension();
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

    void VisualizeRaycasts()
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

    private void Suspension()
    {
        foreach (GameObject wheel in rayPoints)
        {
            // Shoot a ray down

            RaycastHit hit;

            float wheelRestDistance = wheelRadius + suspensionDistance;

            if (Physics.Raycast(wheel.transform.position, -Vector3.up, out hit, wheelRestDistance))
            {
                float wheelOffset = hit.distance - wheelRestDistance;
                Debug.Log($"Ground is {hit.distance} meters below {wheel.name}. Wheel pushed up {wheelOffset}");
                Debug.DrawLine(wheel.transform.position, hit.point, Color.magenta);

                Vector3 wheelWorldVel = rigidbody.GetPointVelocity(wheel.transform.position);

                float offset = wheelRestDistance - hit.distance;
                float vel = Vector3.Dot(wheel.transform.up, wheelWorldVel);

                float force = (offset * springStrength) - (vel * springDamper);
                rigidbody.AddForceAtPosition(wheel.transform.up * force, wheel.transform.position);

                Debug.Log($"Adding force {force}");
            }
        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}
