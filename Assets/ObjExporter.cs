using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class ObjExporter : MonoBehaviour
{
    public static void MeshToFile(Mesh mesh, string filePath)
    {
        List<string> lines = new List<string>();

        lines.Add("g " + Path.GetFileNameWithoutExtension(filePath));

        foreach (Vector3 v in mesh.vertices)
            lines.Add(string.Format("v {0} {1} {2}", v.x, v.y, v.z));

        for (int i = 0; i < mesh.triangles.Length; i += 3)
        {
            int i1 = mesh.triangles[i] + 1;
            int i2 = mesh.triangles[i + 1] + 1;
            int i3 = mesh.triangles[i + 2] + 1;

            lines.Add(string.Format("f {0} {1} {2}", i1, i2, i3));
        }

        File.WriteAllLines(Path.Combine("Assets", filePath), lines.ToArray());
    }
}
