using UnityEngine;

[ExecuteAlways]
[RequireComponent(typeof(Terrain))]
public class PerlinTerrainGenerator : MonoBehaviour
{
    [Header("Terrain size")]
    public int heightmapResolution = 513;
    public float terrainWidth = 20f;
    public float terrainLength = 20f;
    public float terrainHeight = 1.5f;

    [Header("Perlin noise")]
    public float noiseScaleX = 8f;
    public float noiseScaleZ = 8f;
    public float offsetX = 0f;
    public float offsetZ = 0f;
    public bool centerAroundZero = true;

    [Header("Generation")]
    public bool generateOnStart = false;

    private Terrain terrainComp;
    private TerrainData terrainData;

    void Start()
    {
        if (generateOnStart)
            GenerateTerrain();
    }

    [ContextMenu("Generate Terrain")]
    public void GenerateTerrain()
    {
        terrainComp = GetComponent<Terrain>();

        if (terrainComp == null)
        {
            Debug.LogError("No Terrain component found.");
            return;
        }

        terrainData = terrainComp.terrainData;
        if (terrainData == null)
        {
            Debug.LogError("No TerrainData found.");
            return;
        }

        terrainData.heightmapResolution = heightmapResolution;
        terrainData.size = new Vector3(terrainWidth, terrainHeight, terrainLength);

        int resolution = terrainData.heightmapResolution;
        float[,] heights = new float[resolution, resolution];

        for (int z = 0; z < resolution; z++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float xCoord = (float)x / (resolution - 1);
                float zCoord = (float)z / (resolution - 1);

                float nx = offsetX + xCoord * noiseScaleX;
                float nz = offsetZ + zCoord * noiseScaleZ;

                float heightValue = Mathf.PerlinNoise(nx, nz);

                if (centerAroundZero)
                {
                    heightValue = 0.5f + (heightValue - 0.5f);
                }

                heights[z, x] = Mathf.Clamp01(heightValue);
            }
        }

        terrainData.SetHeights(0, 0, heights);
    }
}