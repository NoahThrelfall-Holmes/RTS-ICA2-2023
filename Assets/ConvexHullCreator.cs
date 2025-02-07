using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConvexHullMeshCreator
{
    public Mesh CreateConvexHullMesh(List<Vector3> points)
    {
        // Find the initial two points with the minimum and maximum x-coordinates
        Vector3 A = FindMinXPoint(points);
        Vector3 B = FindMaxXPoint(points);

        List<Vector3> leftSet = new List<Vector3>();
        List<Vector3> rightSet = new List<Vector3>();

        // Divide points into two sets based on which side of the line AB they fall on
        foreach (Vector3 point in points)
        {
            if (point == A || point == B) continue;

            if (IsPointLeftOfLine(A, B, point))
                leftSet.Add(point);
            else
                rightSet.Add(point);
        }

        List<Vector3> convexHull = new List<Vector3>();
        convexHull.Add(A);
        convexHull.Add(B);

        // Find hull on both sides
        FindHull(leftSet, A, B, convexHull);
        FindHull(rightSet, B, A, convexHull);

        // Triangulate the convex hull points
        List<int> triangles = TriangulateConvexHull(convexHull);

        // Create a Unity mesh from the convex hull points and triangles
        Mesh mesh = new Mesh();
        mesh.vertices = convexHull.ToArray();
        mesh.triangles = triangles.ToArray();

        return mesh;
    }

    private Vector3 FindMinXPoint(List<Vector3> points)
    {
        Vector3 minXPoint = points[0];
        foreach (Vector3 point in points)
        {
            if (point.x < minXPoint.x)
                minXPoint = point;
        }
        return minXPoint;
    }

    private Vector3 FindMaxXPoint(List<Vector3> points)
    {
        Vector3 maxXPoint = points[0];
        foreach (Vector3 point in points)
        {
            if (point.x > maxXPoint.x)
                maxXPoint = point;
        }
        return maxXPoint;
    }

    private bool IsPointLeftOfLine(Vector3 A, Vector3 B, Vector3 P)
    {
        // Determine whether a point P lies to the left of a line AB
        return ((B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x)) > 0;
    }

    private void FindHull(List<Vector3> points, Vector3 A, Vector3 B, List<Vector3> hull)
    {
        // Implement recursive hull finding (you may need to expand this for 3D cases)
        // A basic 2D implementation is provided for reference
        if (points.Count == 0) return;

        int insertIndex = hull.IndexOf(B);
        float distToB = float.MinValue;

        for (int i = 0; i < points.Count; i++)
        {
            float dist = Vector3.Distance(B, points[i]);

            if (dist > distToB)
            {
                insertIndex = i;
                distToB = dist;
            }
        }

        Vector3 C = points[insertIndex];
        hull.Insert(insertIndex, C);

        List<Vector3> leftSet = new List<Vector3>();
        List<Vector3> rightSet = new List<Vector3>();

        for (int i = 0; i < points.Count; i++)
        {
            if (points[i] == C || points[i] == A) continue;

            if (IsPointLeftOfLine(A, C, points[i]))
                leftSet.Add(points[i]);
            else if (IsPointLeftOfLine(C, B, points[i]))
                rightSet.Add(points[i]);
        }

        FindHull(leftSet, A, C, hull);
        FindHull(rightSet, C, B, hull);
    }

    private List<int> TriangulateConvexHull(List<Vector3> convexHull)
    {
        List<int> triangles = new List<int>();

        int numVertices = convexHull.Count;

        // Ensure that we have at least 3 vertices to form triangles
        if (numVertices < 3)
        {
            Debug.LogWarning("Cannot triangulate with less than 3 vertices.");
            return triangles;
        }

        // Create a list of vertices indices
        List<int> vertexIndices = new List<int>();
        for (int i = 0; i < numVertices; i++)
        {
            vertexIndices.Add(i);
        }

        int count = numVertices;
        int earIndex;

        while (count > 2)
        {
            for (earIndex = 0; earIndex < count; earIndex++)
            {
                int previousIndex = (earIndex + count - 1) % count;
                int currentIndex = earIndex;
                int nextIndex = (earIndex + 1) % count;

                Vector3 previousVertex = convexHull[vertexIndices[previousIndex]];
                Vector3 currentVertex = convexHull[vertexIndices[currentIndex]];
                Vector3 nextVertex = convexHull[vertexIndices[nextIndex]];

                // Check if the current vertex is an ear (no other vertex inside the triangle)
                if (IsEar(previousVertex, currentVertex, nextVertex, convexHull, vertexIndices))
                {
                    // Add the triangle to the list of triangles
                    triangles.Add(vertexIndices[previousIndex]);
                    triangles.Add(vertexIndices[currentIndex]);
                    triangles.Add(vertexIndices[nextIndex]);

                    // Remove the ear vertex
                    vertexIndices.RemoveAt(currentIndex);
                    count--;

                    // Restart the ear check
                    earIndex = -1;
                }
            }
        }

        return triangles;
    }

    private bool IsEar(Vector3 A, Vector3 B, Vector3 C, List<Vector3> convexHull, List<int> vertexIndices)
    {
        // Check if vertex B is an ear (no other vertex inside the triangle ABC)
        for (int i = 0; i < vertexIndices.Count; i++)
        {
            int vertexIndex = vertexIndices[i];
            Vector3 testVertex = convexHull[vertexIndex];

            if (testVertex != A && testVertex != B && testVertex != C)
            {
                if (IsPointInTriangle(A, B, C, testVertex))
                {
                    return false;
                }
            }
        }

        return true;
    }

    private bool IsPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 point)
    {
        // Check if a point is inside a triangle using barycentric coordinates
        Vector3 v0 = C - A;
        Vector3 v1 = B - A;
        Vector3 v2 = point - A;

        float dot00 = Vector3.Dot(v0, v0);
        float dot01 = Vector3.Dot(v0, v1);
        float dot02 = Vector3.Dot(v0, v2);
        float dot11 = Vector3.Dot(v1, v1);
        float dot12 = Vector3.Dot(v1, v2);

        float invDenominator = 1.0f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenominator;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenominator;

        return (u >= 0) && (v >= 0) && (u + v < 1);
    }
}

