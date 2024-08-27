using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class CarControlWithRaycasting : MonoBehaviour
{

    Rigidbody rigidBody;
    [SerializeField] float wheelOffsetX = 0.5f;
    [SerializeField] float wheelOffsetY = 0.0f;
    [SerializeField] float wheelOffsetZ = 1.0f;


    List<GameObject> wheels;

    Camera camera;

    LineRenderer lineRenderer;

    // Start is called before the first frame update
    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        rigidBody = GetComponent<Rigidbody>();

        camera = Camera.main;

    }

    void AddSpringForces()
    {

    }

    // See Order of Execution for Event Functions for information on FixedUpdate() and Update() related to physics queries
    void FixedUpdate()
    {
        AddSpringForces();
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 wheelPosition = transform.position;
        wheelPosition.x += wheelOffsetX;
        wheelPosition.y += wheelOffsetY;
        wheelPosition.z += wheelOffsetZ;

        float vInput = Input.GetAxis("Vertical");
        float hInput = Input.GetAxis("Horizontal");

        Ray ray = new(wheelPosition, -Vector3.up);
        RaycastHit hit;


        if (Physics.Raycast(ray, out hit, 100))
        {
            lineRenderer.enabled = true;
            lineRenderer.SetPosition(0, wheelPosition);
            lineRenderer.SetPosition(1, hit.point);
            Debug.Log(hit.transform.name);
        }
        else
        {
            lineRenderer.enabled = false;
        }

    }
}

