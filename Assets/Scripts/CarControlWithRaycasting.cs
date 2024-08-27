using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarControlWithRaycasting : MonoBehaviour
{

    Rigidbody rigidBody;

    // Start is called before the first frame update
    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {

        float vInput = Input.GetAxis("Vertical");
        float hInput = Input.GetAxis("Horizontal");

        Debug.Log("YEET");
    }
}

