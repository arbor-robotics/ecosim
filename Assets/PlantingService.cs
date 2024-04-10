using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlantingService : MonoBehaviour
{

public GameObject[] seedlingPrefabs;

// Start is called before the first frame update
void Start()
{
}

// Update is called once per frame
void Update()
{
	if (Input.GetKeyDown(KeyCode.Space))
	{
		var i = Random.Range(0, seedlingPrefabs.Length);

		Debug.Log("Planting!");
		GameObject seedling = Instantiate(seedlingPrefabs[i], transform.position, Quaternion.identity);
		seedling.transform.localScale = new Vector3(0.03f, 0.02f, 0.03f);
	}
}
}
