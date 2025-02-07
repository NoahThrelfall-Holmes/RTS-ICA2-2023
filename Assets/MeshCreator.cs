using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshCreator : MonoBehaviour
{
    void Start()
    {
        Mesh mesh = new Mesh();

        Vector3[] vertices = new Vector3[] { new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0) };
        int[] indices = new int[] { 0, 1, 1, 2, 2, 0 };

        mesh.vertices = vertices;
        mesh.triangles = indices;

        mesh.RecalculateNormals();
        mesh.RecalculateTangents();

        ObjExporter.MeshToFile(mesh, "NewMesh.obj");
        
        CreatePrefabFromMesh(mesh);
    }

    void CreatePrefabFromMesh(Mesh mesh)
    {
        GameObject meshObject = new GameObject("NewMeshObject");
        meshObject.AddComponent<MeshFilter>().mesh = mesh;
        meshObject.AddComponent<MeshRenderer>();

        UnityEditor.PrefabUtility.SaveAsPrefabAsset(meshObject, "Assets/NewPrefab.prefab");
        Destroy(meshObject);
    }
}
