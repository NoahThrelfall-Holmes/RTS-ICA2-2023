using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HullMesh
{
    private List<Vector3> inputVertices;

    public HullMesh(List<Vector3> vertices)
    {
        inputVertices = vertices;
    }

    public Mesh CreateConvexHull()
    {
        // Find initial extreme points
        Vector3 maxX, minX, maxY, minY, maxZ, minZ;
        FindExtremePoints(out maxX, out minX, out maxY, out minY, out maxZ, out minZ);

        // Initial partitioning
        Vector3 p1 = maxX, p2 = maxY, p3 = maxZ;
        List<Vector3> abovePlane, belowPlane;
        InitialPartitioning(p1, p2, p3, out abovePlane, out belowPlane);

        // Recursive partitioning and hull construction
        List<Vector3> hullPoints = new List<Vector3>();
        ProcessPartition(abovePlane, p1, p2, p3, hullPoints);
        ProcessPartition(belowPlane, p1, p2, p3, hullPoints);

        // Constructing the mesh
        return CreateHullMesh(hullPoints);
    }

    private void FindExtremePoints(out Vector3 maxX, out Vector3 minX, out Vector3 maxY, out Vector3 minY, out Vector3 maxZ, out Vector3 minZ)
    {
        maxX = minX = maxY = minY = maxZ = minZ = inputVertices[0];
        foreach (var vertex in inputVertices)
        {
            if (vertex.x > maxX.x) maxX = vertex;
            if (vertex.x < minX.x) minX = vertex;
            if (vertex.y > maxY.y) maxY = vertex;
            if (vertex.y < minY.y) minY = vertex;
            if (vertex.z > maxZ.z) maxZ = vertex;
            if (vertex.z < minZ.z) minZ = vertex;
        }
    }

    private void InitialPartitioning(Vector3 p1, Vector3 p2, Vector3 p3, out List<Vector3> abovePlane, out List<Vector3> belowPlane)
    {
        abovePlane = new List<Vector3>();
        belowPlane = new List<Vector3>();
        Vector3 normal = Vector3.Cross(p2 - p1, p3 - p1);
        float d = -Vector3.Dot(normal, p1);
        foreach (var vertex in inputVertices)
        {
            float side = Vector3.Dot(normal, vertex) + d;
            if (side > 0) abovePlane.Add(vertex);
            else belowPlane.Add(vertex);
        }
    }

    void ProcessPartition(List<Vector3> points, Vector3 p1, Vector3 p2, Vector3 p3, List<Vector3> hullPoints)
    {
        if (points.Count == 0) return;

        Vector3 farthestPoint = FindFarthestPoint(points, p1, Vector3.Cross(p2 - p1, p3 - p1));
        hullPoints.Add(farthestPoint);

        // New planes formed by the farthest point and edges of the initial plane
        // Recursively partition and process each subset of points
        // ... (Recursive partitioning logic goes here)
    }

    private Mesh CreateHullMesh(List<Vector3> hullPoints)
    {
        Mesh hullMesh = new Mesh();
        hullMesh.vertices = hullPoints.ToArray();
        // Determine triangles and construct the mesh
        // ...
        hullMesh.RecalculateNormals();
        return hullMesh;
    }

    Vector3 FindFarthestPoint(List<Vector3> points, Vector3 planePoint, Vector3 planeNormal)
    {
        float maxDistance = -Mathf.Infinity;
        Vector3 farthestPoint = Vector3.zero;

        foreach (var point in points)
        {
            float distance = Mathf.Abs(Vector3.Dot(planeNormal, point - planePoint));
            if (distance > maxDistance)
            {
                maxDistance = distance;
                farthestPoint = point;
            }
        }

        return farthestPoint;
    }
}

/*List<Vector3> hullPoints = new List<Vector3> { p1, p2, p3 };
ProcessPartition(abovePlane, p1, p2, p3, hullPoints);
ProcessPartition(belowPlane, p1, p2, p3, hullPoints);

Vector3 farthestAbove = FindFarthestPoint(abovePlane, p1, normal);
Vector3 farthestBelow = FindFarthestPoint(belowPlane, p1, normal);*/
