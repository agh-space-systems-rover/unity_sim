using System.Collections;
using System.Collections.Generic;
using UnityEngine;

#if UNITY_EDITOR
using UnityEditor;
#endif

public class ArUcoTag : MonoBehaviour
{
    public enum Dict
    {
        _4x4_50,
        _5x5_100,
    }

    private Dictionary<Dict, (string, int)> dictInfo = new Dictionary<Dict, (string, int)>
    {
        { Dict._4x4_50, ("4x4_50", 50) },
        { Dict._5x5_100, ("5x5_100", 100) },
    };

    [SerializeField]
    private Dict dict = Dict._4x4_50;
    [SerializeField]
    private int id = 0;
    [SerializeField]
    [Range(0.0F, 2.0F)]
    private float height = 0.5F;
    [SerializeField]
    [Range(0.05F, 1.0F)]
    private float size = 0.1F;

    private Transform pole;
    private Transform cube;

#if UNITY_EDITOR
    void OnValidate()
    {
        pole = transform.Find("Pole");
        cube = transform.Find("Cube");

        // Normalize ID.
        if (id < 0)
        {
            id = 0;
        }
        else if (id >= dictInfo[dict].Item2)
        {
            id = dictInfo[dict].Item2 - 1;
        }

        // Set dimensions of meshes.
        pole.localScale = new Vector3(0.03F, height / 2, 0.03F);
        pole.localPosition = new Vector3(0, height / 2, 0);
        cube.localScale = new Vector3(size, size, size);
        cube.localPosition = new Vector3(0, height + size / 2, 0);

        // Set the Cube material.
        Texture2D texture = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Simulation/ArUco/Textures/" + dictInfo[dict].Item1 + "/" + id + ".png");
        Material material = AssetDatabase.LoadAssetAtPath<Material>("Assets/Simulation/ArUco/ArUcoTagCube.mat");
        material = new Material(material); // Clone the material to avoid modifying the original.
        material.mainTexture = texture;
        cube.GetComponent<MeshRenderer>().material = material;
    }
#endif
}
