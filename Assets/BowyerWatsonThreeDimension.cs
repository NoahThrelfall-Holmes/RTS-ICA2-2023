using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class BowyerWatsonThreeDimensions
{
    private List<Vector3> points;
    private List<Tetrahedron> tetrahedra;
    private List<Vector3> superTetrahedronVertices;

    public BowyerWatsonThreeDimensions(List<Vector3> _points)
    {
        this.points = _points;
        this.tetrahedra = new List<Tetrahedron>();
        this.superTetrahedronVertices = new List<Vector3>();

        // Create an initial tetrahedron to encompass all points
        InitializeSuperTetrahedron(_points);
    }

    private void InitializeSuperTetrahedron(List<Vector3> _inputVertices)
    {
        if (_inputVertices.Count < 4)
        {
            Debug.LogError("At least four input vertices are required.");
            return;
        }

        // Find the minimum and maximum coordinates of the input vertices
        Vector3 minPoint = _inputVertices[0];
        Vector3 maxPoint = _inputVertices[0];

        foreach (Vector3 vertex in _inputVertices)
        {
            minPoint = Vector3.Min(minPoint, vertex);
            maxPoint = Vector3.Max(maxPoint, vertex);
        }

        // Calculate the size of the bounding box
        Vector3 size = maxPoint - minPoint;

        // Increase the size to ensure it covers all points
        size *= 2;

        // Define the vertices of the super tetrahedron
        Vector3 vertexA = minPoint - new Vector3(size.x, 0, 0);
        Vector3 vertexB = minPoint + new Vector3(size.x, 0, 0);
        Vector3 vertexC = minPoint + new Vector3(0, size.y, 0);
        Vector3 vertexD = minPoint + new Vector3(0, 0, size.z);

        // Add the vertices of the super tetrahedron to the list
        superTetrahedronVertices.Add(vertexA);
        superTetrahedronVertices.Add(vertexB);
        superTetrahedronVertices.Add(vertexC);
        superTetrahedronVertices.Add(vertexD);

        // Create the super tetrahedron
        Tetrahedron superTetrahedron = new Tetrahedron(vertexA, vertexB, vertexC, vertexD);

        // Add the super tetrahedron to the list of tetrahedra
        tetrahedra.Add(superTetrahedron);
    }

    public void Triangulate()
    {
        foreach (var point in points)
        {
            // Find tetrahedra whose circumsphere contains the point
            var badTetrahedra = FindBadTetrahedra(point);

            // Remove bad tetrahedra from the triangulation
            RemoveTetrahedra(badTetrahedra);

            // Create new tetrahedra by connecting the point to the faces of the bad tetrahedra
            CreateNewTetrahedra(point, badTetrahedra);
        }

        // Remove tetrahedra that share vertices with the original super tetrahedron
        RemoveSuperTetrahedronTetrahedra();
    }

    private void RemoveTetrahedra(List<Tetrahedron> badTetrahedra)
    {
        foreach (var badTetra in badTetrahedra)
        {
            tetrahedra.Remove(badTetra);
        }
    }

    private void RemoveSuperTetrahedronTetrahedra()
    {
        tetrahedra.RemoveAll(tetra => tetra.ContainsAny(superTetrahedronVertices));
    }

    private List<Tetrahedron> FindBadTetrahedra(Vector3 point)
    {
        List<Tetrahedron> badTetrahedra = new List<Tetrahedron>();

        foreach (var tetra in tetrahedra)
        {
            if (IsPointInsideCircumsphere(point, tetra))
            {
                badTetrahedra.Add(tetra);
            }
        }

        return badTetrahedra;
    }

    private bool IsPointInsideCircumsphere(Vector3 point, Tetrahedron tetrahedron)
    {
        // Calculate the circumsphere of the tetrahedron
        Vector3 circumcenter = CalculateCircumcenter(tetrahedron);
        float circumradius = CalculateCircumradius(tetrahedron, circumcenter);

        // Check if the point lies inside the circumsphere
        float distanceSquared = (point - circumcenter).sqrMagnitude;
        return distanceSquared <= (circumradius * circumradius);
    }

    private Vector3 CalculateCircumcenter(Tetrahedron tetrahedron)
    {
        Vector3 a = tetrahedron.vertices[0];
        Vector3 b = tetrahedron.vertices[1];
        Vector3 c = tetrahedron.vertices[2];
        Vector3 d = tetrahedron.vertices[3];

        // Calculate the coefficients of the system
        float bx = b.x - a.x;
        float by = b.y - a.y;
        float bz = b.z - a.z;

        float cx = c.x - a.x;
        float cy = c.y - a.y;
        float cz = c.z - a.z;

        float dx = d.x - a.x;
        float dy = d.y - a.y;
        float dz = d.z - a.z;

        // Calculate the determinants
        float bx_cy = bx * cy;
        float by_cx = by * cx;

        float bx_cz = bx * cz;
        float bz_cx = bz * cx;

        float by_cz = by * cz;
        float bz_cy = bz * cy;

        float dx_cy = dx * cy;
        float dy_cx = dy * cx;

        float dx_cz = dx * cz;
        float dz_cx = dz * cx;

        float dy_cz = dy * cz;
        float dz_cy = dz * cy;

        // Calculate the squares of distances
        float a2 = a.x * a.x + a.y * a.y + a.z * a.z;
        float b2 = b.x * b.x + b.y * b.y + b.z * b.z;
        float c2 = c.x * c.x + c.y * c.y + c.z * c.z;
        float d2 = d.x * d.x + d.y * d.y + d.z * d.z;

        // Solve the system of equations
        float nominatorX = (b2 - a2) * (cy * dz - cz * dy) +
                           (c2 - a2) * (dy * bz - dz * by) +
                           (d2 - a2) * (by * cz - bz * cy);

        float nominatorY = (b2 - a2) * (cz * dx - cx * dz) +
                           (c2 - a2) * (dz * bx - dx * bz) +
                           (d2 - a2) * (bz * cx - bx * cz);

        float nominatorZ = (b2 - a2) * (cx * dy - cy * dx) +
                           (c2 - a2) * (dx * by - dy * bx) +
                           (d2 - a2) * (bx * cy - by * cx);

        float denominator = 2 * (bx * (cy * dz - cz * dy) +
                                 by * (cz * dx - cx * dz) +
                                 bz * (cx * dy - cy * dx));

        Vector3 circumcenter = new Vector3(
            nominatorX / denominator,
            nominatorY / denominator,
            nominatorZ / denominator
        );

        return circumcenter;
    }


    private float CalculateCircumradius(Tetrahedron tetrahedron, Vector3 circumcenter)
    {
        // The circumradius is the distance from the circumcenter to any vertex of the tetrahedron
        return Vector3.Distance(tetrahedron.vertices[0], circumcenter);
    }

    private void CreateNewTetrahedra(Vector3 point, List<Tetrahedron> badTetrahedra)
    {
        // A set to keep track of all faces of the bad tetrahedra
        HashSet<Triangle3D> allFaces = new HashSet<Triangle3D>();

        // A dictionary to count the occurrences of each face
        Dictionary<Triangle3D, int> faceOccurrences = new Dictionary<Triangle3D, int>();

        // Collect faces of bad tetrahedra and count their occurrences
        foreach (var badTetra in badTetrahedra)
        {
            for (int i = 0; i < 4; i++)
            {
                Triangle3D face = badTetra.GetFace(i);
                allFaces.Add(face);

                if (faceOccurrences.ContainsKey(face))
                {
                    faceOccurrences[face] += 1; // Face is shared
                }
                else
                {
                    faceOccurrences[face] = 1; // First occurrence of face
                }
            }
        }

        // Determine external faces (those that are not shared)
        List<Triangle3D> externalFaces = allFaces
                                          .Where(face => faceOccurrences[face] == 1)
                                          .ToList();

        // Create new tetrahedra from external faces
        foreach (var face in externalFaces)
        {
            Tetrahedron newTetra = new Tetrahedron(face.vertices[0], face.vertices[1], face.vertices[2], point);
            tetrahedra.Add(newTetra);
        }
    }



    private bool IsFaceShared(Triangle3D face, List<Tetrahedron> tetrahedra)
    {
        int sharedCount = 0;

        foreach (var tetra in tetrahedra)
        {
            for (int i = 0; i < 4; i++)
            {
                if (face.Equals(tetra.GetFace(i)))
                {
                    sharedCount++;
                    if (sharedCount > 1)
                    {
                        return true; // The face is shared by more than one tetrahedron
                    }
                    break; // No need to check other faces of the same tetrahedron
                }
            }
        }

        return false; // The face is unique to one tetrahedron
    }

    public Mesh CreateMesh(Vector3 centralFeaturePoint)
    {
        Mesh mesh = new Mesh();
        List<Vector3> meshVertices = new List<Vector3>();
        List<int> meshTriangles = new List<int>();

        // Create a mapping from Vector3 to index
        Dictionary<Vector3, int> vertexIndexMapping = new Dictionary<Vector3, int>();
        int index = 0;

        foreach (var tetra in tetrahedra)
        {
            foreach (var vertex in tetra.vertices)
            {
                if (!vertexIndexMapping.ContainsKey(vertex))
                {
                    vertexIndexMapping[vertex] = index++;
                    meshVertices.Add(vertex);
                }
            }

            // Add triangles (faces) of the tetrahedron to the mesh
            for (int i = 0; i < 4; i++)
            {
                Triangle3D face = tetra.GetFace(i);

                // Calculate the normal of the face
                Vector3 edge1 = face.vertices[1] - face.vertices[0];
                Vector3 edge2 = face.vertices[2] - face.vertices[0];
                Vector3 normal = Vector3.Cross(edge1, edge2).normalized;

                // Calculate the vector from the central feature point to one of the vertices
                Vector3 vectorToVertex = face.vertices[0] - centralFeaturePoint;

                // Check the dot product to determine if the normal faces away from the central feature point
                if (Vector3.Dot(normal, vectorToVertex) > 0)
                {
                    // Add the triangle to the mesh
                    meshTriangles.Add(vertexIndexMapping[face.vertices[0]]);
                    meshTriangles.Add(vertexIndexMapping[face.vertices[1]]);
                    meshTriangles.Add(vertexIndexMapping[face.vertices[2]]);
                }
            }
        }

        // Filter out internal triangles using a method to check if the triangle is part of the convex hull
        FilterInternalTriangles(mesh, centralFeaturePoint);

        mesh.vertices = meshVertices.ToArray();
        mesh.triangles = meshTriangles.ToArray();
        mesh.RecalculateNormals();

        return mesh;
    }

    private void FilterInternalTriangles(Mesh mesh, Vector3 centralFeaturePoint)
    {
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        List<int> validTriangles = new List<int>();

        for (int i = 0; i < triangles.Length; i += 3)
        {
            int vertexIndex0 = triangles[i];
            int vertexIndex1 = triangles[i + 1];
            int vertexIndex2 = triangles[i + 2];

            Vector3 vertex0 = vertices[vertexIndex0];
            Vector3 vertex1 = vertices[vertexIndex1];
            Vector3 vertex2 = vertices[vertexIndex2];

            // Calculate the normal of the triangle
            Vector3 edge1 = vertex1 - vertex0;
            Vector3 edge2 = vertex2 - vertex0;
            Vector3 normal = Vector3.Cross(edge1, edge2).normalized;

            // Calculate the vector from the central feature point to one of the vertices
            Vector3 vectorToVertex = vertex0 - centralFeaturePoint;

            // Check the dot product to determine if the normal faces away from the central feature point
            if (Vector3.Dot(normal, vectorToVertex) >= 0)
            {
                // This triangle is facing away from or parallel to the central feature point
                // Add it to the list of valid triangles
                validTriangles.Add(vertexIndex0);
                validTriangles.Add(vertexIndex1);
                validTriangles.Add(vertexIndex2);
            }
        }

        // Update the mesh with the filtered triangles
        mesh.triangles = validTriangles.ToArray();
    }
}


public class Tetrahedron
{
    public List<Vector3> vertices;

    public Tetrahedron(Vector3 vertex1, Vector3 vertex2, Vector3 vertex3, Vector3 vertex4)
    {
        vertices = new List<Vector3> { vertex1, vertex2, vertex3, vertex4 };
    }

    public Triangle3D GetFace(int index)
    {
        switch (index)
        {
            case 0:
                return new Triangle3D(vertices[0], vertices[1], vertices[2]);
            case 1:
                return new Triangle3D(vertices[0], vertices[1], vertices[3]);
            case 2:
                return new Triangle3D(vertices[0], vertices[2], vertices[3]);
            case 3:
                return new Triangle3D(vertices[1], vertices[2], vertices[3]);
            default:
                throw new ArgumentOutOfRangeException("index", "Index must be between 0 and 3");
        }
    }

    public bool ContainsAny(List<Vector3> otherVertices)
    {
        foreach (var vertex in vertices)
        {
            if (otherVertices.Contains(vertex))
            {
                return true;
            }
        }
        return false;
    }
}

public class Edge3D
{
    public Vector3 Start { get; }
    public Vector3 End { get; }

    public Edge3D(Vector3 start, Vector3 end)
    {
        Start = start;
        End = end;
    }
}

public class Triangle3D
{
    public List<Vector3> vertices;

    public Triangle3D(Vector3 vertex1, Vector3 vertex2, Vector3 vertex3)
    {
        vertices = new List<Vector3>
        {
            vertex1,
            vertex2,
            vertex3
        };

    }
}