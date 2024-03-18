using UnityEngine;
using UnityEditor;
using System.IO;

class HeightmapExportJPG : EditorWindow
{
static TerrainData terraindata;


[MenuItem("Window/Terrain to image")]
static void Init()
{
	terraindata = null;
	Terrain terrain = null;

	// terrain = GameObject.Find("Terrain_2_2_b6e03cf0-de91-4cae-b6af-0128d856ae43").GetComponent<Terrain>();

	if (Selection.activeGameObject)
		terrain = Selection.activeGameObject.GetComponent<Terrain>();

	if (!terrain)
	{
		terrain = Terrain.activeTerrain;
	}
	if (terrain)
	{
		terraindata = terrain.terrainData;
	}
	if (terraindata == null)
	{
		EditorUtility.DisplayDialog("No terrain selected", "Please select a terrain.", "Cancel");
		return;
	}

	//// get the terrain heights into an array and apply them to a texture2D
	Texture2D duplicateHeightMap = new Texture2D(terraindata.heightmapResolution, terraindata.heightmapResolution, TextureFormat.ARGB32, false);
	float[,] rawHeights = terraindata.GetHeights(0, 0, terraindata.heightmapResolution, terraindata.heightmapResolution);

	Debug.Log($"Terrain has dims {terraindata.heightmapResolution}");
	Debug.Log($"Duplicate HM has dims {duplicateHeightMap.height} x {duplicateHeightMap.width}");

	/// run through the array row by row to find max and min heights
	// float maxHeight = rawHeights.Max();
	float minHeight = 9999f;
	float maxHeight = -9999f;

	foreach (var height in rawHeights)
	{
		if (height > maxHeight)
			maxHeight = height;
		if (height < minHeight)
			minHeight = height;
	}
	Debug.Log($"Bounds were ({minHeight}, {maxHeight})");

	int TERRAIN_HEIGHT = 800; // this can be found in the terrain settings menu
	Debug.Log($"Min height: {minHeight*TERRAIN_HEIGHT} meters, max {maxHeight*TERRAIN_HEIGHT} meters");

	for (int y = 0; y < duplicateHeightMap.height; y++)
	{
		for (int x = 0; x < duplicateHeightMap.width; x++)
		{
			float pixel = (rawHeights[x,y] - minHeight) / maxHeight;
			/// for wach pixel set RGB to the same so it's gray
			Color color = new Vector4(pixel, pixel, pixel, 1);
			duplicateHeightMap.SetPixel(y, x, color);
		}
	}

	// Apply all SetPixel calls
	duplicateHeightMap.Apply();

	// Encode the texture in JPG format
	byte[] bytes = ImageConversion.EncodeToJPG(duplicateHeightMap);
	Object.Destroy(duplicateHeightMap);

	// Write the returned byte array to a file in the project folder
	// File.WriteAllBytes(Application.dataPath + "/../SavedScreen.jpg", bytes);

	string path = EditorUtility.SaveFilePanel(
		$"Save {terrain.name} as",
		"",
		$"{terrain.name}.jpg",
		"jpg");

	// var extension = Path.GetExtension(path);
	// byte[] pngData = null;// duplicateHeightMap.EncodeToPNG();

	// switch(extension)
	// {
	// case ".jpg":
	// 	pngData = duplicateHeightMap.EncodeToJPG();
	// 	break;

	// case ".png":
	// 	pngData = duplicateHeightMap.EncodeToPNG();
	// 	break;
	// }

	if (bytes != null)
	{
		File.WriteAllBytes(path, bytes);
		EditorUtility.DisplayDialog("Heightmap Duplicated", "Saved in " + path, "Done");
	}else
	{
		EditorUtility.DisplayDialog("Error", "Failed to save height map as JPG", "OK");
	}

	// AssetDatabase.Refresh();
}
}