using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEngine.UI.Image;

// Copyright - me
// Dear Matthew, my scincere apologies for the lack of comments. I tend to miss them out in the spur of the moment. I have retroactively added some
// however I usually focus on having longer, more descriptive, variable names to make up for it. I hope you dont have too much trouble. 

struct Voxel
{
    public Vector3 pos;
    public Vector3 centre;
    public float size;
    public List<Vector3> master_featurePoints; // the feature points this voxel is slave to/owned by/connected to

    public Voxel(Vector3 _pos, float _size)
    {
        pos = _pos;
        size = _size;
        centre = pos + new Vector3(size / 2, size / 2, size / 2);
        master_featurePoints = new List<Vector3>();
    }

    public Voxel(float _size, Vector3 _centre)
    {
        size = _size;
        centre = _centre;
        pos = centre - new Vector3(size / 2, size / 2, size / 2);
        master_featurePoints = new List<Vector3>();
    }
}

public class FractureMaker : MonoBehaviour
{
    [Tooltip("Only cubes work"), Header("Feature Points")]
    [SerializeField] GameObject objectToFracture;
    List<Vector3> cubeCorners = new List<Vector3>();
    [SerializeField] int numOfFeaturePoints;
    List<Vector3> featurePoints = new List<Vector3>();

    [Header("Fracture")]
    [SerializeField] float sizeOfVoxels;
    [SerializeField] float distanceCheckTolerance;
    List<Voxel> activeVoxels = new List<Voxel>();

    List<Voxel> vertexVoxels = new List<Voxel>();
    List<Voxel> potentialVertices = new List<Voxel>();

    List<Voxel> edgeVoxels = new List<Voxel>();
    List<Voxel> edgeVoxelsOnEdge = new List<Voxel>();

    List<Voxel> planeVoxels = new List<Voxel>();
    List<Voxel> planeVoxelsPotentialVertices = new List<Voxel>();

    [Header("Debug"), Range(0, 15)]
    public int featurePointToDisplay = 0;
    [SerializeField] bool shouldCreateMeshes = false;

    void Start()
    {
        DateTime startTime = DateTime.Now;

        CreateFetaurePoints(objectToFracture);
        Fracture(objectToFracture);

        int count = 0;
        foreach (Vector3 fp in featurePoints)
        {
            List<Vector3> inputVertices = new List<Vector3>();

            foreach (Voxel voxel in vertexVoxels)
            {
                if (voxel.master_featurePoints.Contains(fp))
                {
                    inputVertices.Add(voxel.centre);
                }
            }

            if (shouldCreateMeshes)
            {
                BowyerWatsonThreeDimensions bowyerWatsonMeshGen = new BowyerWatsonThreeDimensions(inputVertices);
                // Perform the Delaunay Triangulation
                bowyerWatsonMeshGen.Triangulate();
                // Create Delauney Mesh
                Mesh newMesh = bowyerWatsonMeshGen.CreateMesh(fp);

                // Call CreateConvexHullMesh to get the convex hull as a Unity mesh
                //ConvexHullMeshCreator convexHullMeshCreator = new ConvexHullMeshCreator();
                //Mesh newMesh = convexHullMeshCreator.CreateConvexHullMesh(inputVertices);

                // Create a GameObject to visualize the convex hull
                GameObject convexHullObject = new GameObject("fp" + count.ToString() + "AreaMesh");
                convexHullObject.transform.parent = this.transform; // Set the parent
                convexHullObject.AddComponent<MeshFilter>().mesh = newMesh;
                convexHullObject.AddComponent<MeshRenderer>().material = new Material(Shader.Find("Standard"));
            }

            count++;
        }


        DateTime endTime = DateTime.Now;
        TimeSpan executionTime = endTime - startTime;

        Debug.Log("Execution Time: " + executionTime.TotalMilliseconds + " ms");
    }



    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnDrawGizmos()
    {
        foreach (Vector3 point in featurePoints)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(point, .3f);
        }

        Vector3 voxelSize = new Vector3(sizeOfVoxels, sizeOfVoxels, sizeOfVoxels);
        
        // opposite order for drawing so that edges + vertices are always drawn on top
        foreach (Voxel voxel in planeVoxels)
        {
            // I convert to list in the drawing so I can use .Contains() I am not worried about performance here since this is debug (seemingly very fast regardless)
            List<Vector3> fps = new List<Vector3>();
            foreach (Vector3 fp in voxel.master_featurePoints)
            { fps.Add(fp); }

            // Only render slave voxels of this fp. I found this very useful when debugging
            if (fps.Contains(featurePoints[featurePointToDisplay]))
            {
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireCube(voxel.centre, voxelSize);
            }
        }
        foreach (Voxel voxel in edgeVoxels)
        {
            List<Vector3> fps = new List<Vector3>();
            foreach (Vector3 fp in voxel.master_featurePoints)
            { fps.Add(fp); }
            if (fps.Contains(featurePoints[featurePointToDisplay]))
            {
                Gizmos.color = Color.cyan;
                Gizmos.DrawWireCube(voxel.centre, voxelSize);
            }
        }
        foreach (Voxel voxel in planeVoxelsPotentialVertices)
        {
            List<Vector3> fps = new List<Vector3>();
            foreach (Vector3 fp in voxel.master_featurePoints)
            { fps.Add(fp); }
            if (fps.Contains(featurePoints[featurePointToDisplay]))
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireCube(voxel.centre, voxelSize);

                //foreach (Vector3 fp in fps)
                //{ Debug.DrawLine(voxel.centre, fp); }
            }
        }
        foreach (Voxel voxel in vertexVoxels)
        {
            List<Vector3> fps = new List<Vector3>();
            foreach (Vector3 fp in voxel.master_featurePoints)
            { fps.Add(fp); }
            if (fps.Contains(featurePoints[featurePointToDisplay]))
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawWireCube(voxel.centre, voxelSize);

                //foreach (Vector3 fp in fps)
                //{ Debug.DrawLine(voxel.centre, fp); }
                
            }
            else
            {
                Gizmos.color = Color.white;
                Gizmos.DrawWireCube(voxel.centre, voxelSize);
            }
        }

        
    }

    void Fracture(GameObject _target)
    {
        // slightly reduces computation of the embedded (probably insignificant performance gains but it does make the for statements nicer to look at)
        float posX = _target.transform.position.x;
        float posY = _target.transform.position.y;
        float posZ = _target.transform.position.z;
        float xHelper = _target.transform.localScale.x / 2;
        float yHelper = _target.transform.localScale.y / 2;
        float zHelper = _target.transform.localScale.z / 2;

        float spillOverHelper = sizeOfVoxels / 2; // used for experimenting with the clipping plane method

        //List<Voxel> potentialVertices = new List<Voxel>();

        for (float z = posZ - zHelper - spillOverHelper; z < posZ + zHelper + spillOverHelper; z += sizeOfVoxels)
        {
            for (float x = posX - xHelper - spillOverHelper; x < posX + xHelper + spillOverHelper; x += sizeOfVoxels)
            {
                for (float y = posY - yHelper - spillOverHelper; y < posY + yHelper + spillOverHelper; y += sizeOfVoxels)
                {
                    Vector3 newPos = new Vector3(x, y, z);
                    Voxel newVoxel = new Voxel(newPos, sizeOfVoxels);

                    Dictionary<Vector3, float> fpDistMap = new Dictionary<Vector3, float>(); // Dictionary for distances between this voxel and all feature points. Means I only have to do dist check once
                    foreach (Vector3 fp in featurePoints)
                    {
                        fpDistMap.Add(fp, Vector3.Distance(newVoxel.pos, fp));
                    }

                    float minDistance = float.MaxValue;
                    Vector3 closestFeaturePoint = Vector3.zero;
                    foreach (var entry in fpDistMap)
                    {
                        if (entry.Value < minDistance)
                        {
                            minDistance = entry.Value;
                            closestFeaturePoint = entry.Key;
                        }
                    }

                    foreach (var entry in fpDistMap)
                    {
                        // add and skip
                        if (closestFeaturePoint == entry.Key)
                        {
                            newVoxel.master_featurePoints.Add(closestFeaturePoint);
                            continue;
                        }

                        float distanceDifference = Math.Abs(minDistance - entry.Value);

                        if (distanceDifference <= distanceCheckTolerance)
                        {
                            newVoxel.master_featurePoints.Add(entry.Key);
                        }
                    }

                    if (newVoxel.master_featurePoints.Count == 4)
                    {
                        potentialVertices.Add(newVoxel);
                    }
                    else if (newVoxel.master_featurePoints.Count == 3)
                    {
                        edgeVoxels.Add(newVoxel);
                    }
                    else if (newVoxel.master_featurePoints.Count == 2)
                    {
                        planeVoxels.Add(newVoxel);
                    }
                }
            }
        }

        // get cube corner vertices
        cubeCorners = GetCubeCorners(_target);
        //

        // add cube corners as vertices
        foreach (Vector3 corner in cubeCorners)
        {
            Vector3 closestFeaturePoint = Vector3.zero;
            float minDistance = float.MaxValue;

            foreach (Vector3 fp in featurePoints)
            {
                float distance = Vector3.Distance(corner, fp);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestFeaturePoint = fp;
                }
            }

            Voxel newVoxel = new Voxel(sizeOfVoxels, corner);
            newVoxel.master_featurePoints.Add(closestFeaturePoint);
            vertexVoxels.Add(newVoxel);
        }
        //

        // Check if the same vertex found twice
        CullDuplicateVertices();
        //

        // find edge voxels on edge of the cube
        for (int i = 0; i < edgeVoxels.Count; i++)
        {
            Vector3 voxelCentre = edgeVoxels[i].centre;
            // check if the voxel is by the edge of the cube, within tolerance
            bool isOnEdge = (voxelCentre.x <= posX - xHelper + distanceCheckTolerance || voxelCentre.x >= posX + xHelper - distanceCheckTolerance) ||
                            (voxelCentre.y <= posY - yHelper + distanceCheckTolerance || voxelCentre.y >= posY + yHelper - distanceCheckTolerance) ||
                            (voxelCentre.z <= posZ - zHelper + distanceCheckTolerance || voxelCentre.z >= posZ + zHelper - distanceCheckTolerance);

            // +, +, + == back, right, top
            // -, -, - == front, left, bottom
            if (isOnEdge)
            {
                edgeVoxelsOnEdge.Add(edgeVoxels[i]);
            }
        }
        CullDuplicateEdgeVertices(_target);
        //

        // find plane voxels in the folds and corners of the cube (TODO: reject corners? already have corner verts)
        for (int i = 0; i < planeVoxels.Count; i++)
        {
            Vector3 voxelCentre = planeVoxels[i].centre;

            // check if the voxel is intersecting with the cube's faces in x and y
            bool onXYEdge = ((voxelCentre.x == posX - xHelper || voxelCentre.x == posX + xHelper) &&
                             (voxelCentre.y == posY - yHelper || voxelCentre.y == posY + yHelper) &&
                             (voxelCentre.z > posZ - zHelper && voxelCentre.z < posZ + zHelper));

            // check if the voxel is intersecting with the cube's faces in x and z
            bool onXZEdge = ((voxelCentre.x == posX - xHelper || voxelCentre.x == posX + xHelper) &&
                             (voxelCentre.z == posZ - zHelper || voxelCentre.z == posZ + zHelper) &&
                             (voxelCentre.y > posY - yHelper && voxelCentre.y < posY + yHelper));

            // check if the voxel is intersecting with the cube's faces in y and z
            bool onYZEdge = ((voxelCentre.y == posY - yHelper || voxelCentre.y == posY + yHelper) &&
                             (voxelCentre.z == posZ - zHelper || voxelCentre.z == posZ + zHelper) &&
                             (voxelCentre.x > posX - xHelper && voxelCentre.x < posX + xHelper));

            if (onXYEdge || onXZEdge || onYZEdge)
            {
                planeVoxelsPotentialVertices.Add(planeVoxels[i]);
            }
        }
        CullDuplicatePlaneVertices(_target);
        //

        
    }

    void CullDuplicateVertices()
    {
        // store of unique values, after much fucking around with more simple techniques trying to brute force a magical solution I did some research into similar problems and I found this concept. 
        // Hashmaps are a sort of meme in the programming world that ive seen around but its not something I typicially use, it did fundamentally change the way I approached this problem so I will
        // play around with them some more and slowly add them to my problem solving arsenal
        HashSet<Voxel> comparedVoxels = new HashSet<Voxel>();

        // cull duplicate vertices
        for (int i = 0; i < potentialVertices.Count; i++)
        {
            Voxel currentVoxel = potentialVertices[i];

            if (comparedVoxels.Contains(currentVoxel))
                continue;

            bool isSimilarFound = false;

            for (int j = 0; j < potentialVertices.Count; j++)
            {
                if (i == j) continue;

                Voxel otherVoxel = potentialVertices[j];

                if (AreFourFeaturePointsShared(currentVoxel, otherVoxel)) // I already had the function from a previous attempt so I just reused it, doesnt really have to be a fucntion
                {
                    isSimilarFound = true;
                    comparedVoxels.Add(otherVoxel);
                }
            }

            if (!isSimilarFound || !vertexVoxels.Contains(currentVoxel))
            {
                vertexVoxels.Add(currentVoxel);
                comparedVoxels.Add(currentVoxel);
            }
        }
    }

    bool AreFourFeaturePointsShared(Voxel _a, Voxel _b)
    {
        int sharedPoints = 0;

        foreach (Vector3 pointA in _a.master_featurePoints)
        {
            foreach (Vector3 pointB in _b.master_featurePoints)
            {
                if (pointA == pointB)
                {
                    sharedPoints++;

                    if (sharedPoints >= 4)
                        return true;
                }
            }
        }

        // Return false if fewer than four points are shared
        return false;
    }

    void CullDuplicateEdgeVertices(GameObject _target)
    {
        HashSet<Voxel> addedEdgeVoxels = new HashSet<Voxel>(); // Tracks added edge voxels

        foreach (Voxel currentEdgeVoxel in edgeVoxelsOnEdge)
        {
            bool isSimilarFound = false;

            foreach (Voxel addedVoxel in addedEdgeVoxels)
            {
                if (AreThreeFeaturePointsShared(currentEdgeVoxel, addedVoxel) && !AreOnDifferentFaces(currentEdgeVoxel, addedVoxel, _target))
                {
                    isSimilarFound = true;
                    break; // Found a similar voxel on the same face, no need to check further
                }
            }

            // Add the voxel to vertexVoxels if it's unique or the first in its group on an edge
            if (!isSimilarFound && !vertexVoxels.Contains(currentEdgeVoxel))
            {
                vertexVoxels.Add(currentEdgeVoxel);
                addedEdgeVoxels.Add(currentEdgeVoxel); // Mark this voxel as added
            }
        }
    }

    bool AreOnDifferentFaces(Voxel voxel1, Voxel voxel2, GameObject _target)
    {
        // Logic to determine if voxel1 and voxel2 are on different faces of the cube.
        // This could involve checking if the voxel centers lie on different planes
        // defined by the cube's faces. Since a cube's face can be defined by a normal vector
        // and a point on the face (vertex), you can compare the signed distances from the voxel centers
        // to the plane to determine if they are on different faces.

        // Retrieve the cube's transform information
        Vector3 cubeCenter = _target.transform.position;
        Vector3 cubeSize = _target.transform.localScale;

        // Calculate normals for each face of the cube
        Vector3[] faceNormals = {
        _target.transform.forward,
        _target.transform.right,
        _target.transform.up
    };

        // Check if voxel centers are on opposite sides of any face of the cube
        foreach (Vector3 normal in faceNormals)
        {
            float sign1 = Mathf.Sign(Vector3.Dot(voxel1.centre - cubeCenter, normal));
            float sign2 = Mathf.Sign(Vector3.Dot(voxel2.centre - cubeCenter, normal));

            // If signs are different, voxels are on opposite sides of this face plane
            if (sign1 != sign2)
                return true;
        }

        // If none of the face plane checks were successful, voxels are on the same face
        return false;
    }

    bool AreThreeFeaturePointsShared(Voxel _a, Voxel _b)
    {
        int sharedPoints = 0;

        foreach (Vector3 pointA in _a.master_featurePoints)
        {
            foreach (Vector3 pointB in _b.master_featurePoints)
            {
                if (pointA == pointB)
                {
                    sharedPoints++;

                    if (sharedPoints > 3) // If more than three shared points, return false
                        return false;
                }
            }
        }

        // Return true if exactly three points are shared, false otherwise
        return sharedPoints == 3;
    }

    void CullDuplicatePlaneVertices(GameObject _target)
    {
        HashSet<Voxel> addedVoxels = new HashSet<Voxel>(); // Tracks added voxels

        foreach (Voxel currentPlaneVoxel in planeVoxelsPotentialVertices)
        {
            // Check if the current voxel shares a fold and two feature points with any already added voxel
            bool isSimilarFound = addedVoxels.Any(addedVoxel =>
                !AreInDifferentFolds(currentPlaneVoxel, addedVoxel, _target) &&
                AreTwoFeaturePointsShared(currentPlaneVoxel, addedVoxel));

            // Add the voxel to vertexVoxels if it's either unique or the first in its group
            if (!isSimilarFound && !vertexVoxels.Contains(currentPlaneVoxel))
            {
                vertexVoxels.Add(currentPlaneVoxel);
                addedVoxels.Add(currentPlaneVoxel); // Mark this voxel as added
            }
        }
    }

    bool AreInDifferentFolds(Voxel voxel1, Voxel voxel2, GameObject _target)
    {
        // Define the cube's vertices based on its position and scale
        List<Vector3> cubeVertices = GetCubeCorners(_target);  // This function needs to be defined as previously described

        // Get the closest edge to each voxel by examining proximity to each pair of cube vertices
        int closestEdgeIndex1 = GetClosestEdgeIndex(voxel1.centre, cubeVertices);
        int closestEdgeIndex2 = GetClosestEdgeIndex(voxel2.centre, cubeVertices);

        // Voxels are on different edges if they have different edge indices
        return closestEdgeIndex1 != closestEdgeIndex2;
    }

    int GetClosestEdgeIndex(Vector3 voxelCentre, List<Vector3> cubeVertices)
    {
        // Define the pairs of indices from cubeVertices that make up the cube's 12 edges
        int[][] edgePairs = new int[][]
        {
        new[] {0, 1}, new[] {1, 3}, new[] {3, 2}, new[] {2, 0}, // bottom face edges
        new[] {4, 5}, new[] {5, 7}, new[] {7, 6}, new[] {6, 4}, // top face edges
        new[] {0, 4}, new[] {1, 5}, new[] {2, 6}, new[] {3, 7}  // side edges connecting top and bottom faces
        };

        float minDistance = float.MaxValue;
        int closestEdgeIndex = -1;

        for (int i = 0; i < edgePairs.Length; i++)
        {
            Vector3 closestPoint = ClosestPointOnLineSegment(cubeVertices[edgePairs[i][0]], cubeVertices[edgePairs[i][1]], voxelCentre);
            float distance = Vector3.Distance(voxelCentre, closestPoint);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestEdgeIndex = i;
            }
        }

        return closestEdgeIndex;
    }

    Vector3 ClosestPointOnLineSegment(Vector3 a, Vector3 b, Vector3 p)
    {
        Vector3 ap = p - a;
        Vector3 ab = b - a;
        float magnitudeAB = ab.sqrMagnitude;  // Length of AB vector (it's squareroot)
        float ABAPproduct = Vector3.Dot(ap, ab);  // The DOT product of a_to_p and a_to_b
        float distance = ABAPproduct / magnitudeAB; // The normalized "distance" from a to your closest point

        // Check if P projection is over line segment
        if (distance < 0)
        {
            return a;
        }
        else if (distance > 1)
        {
            return b;
        }
        else
        {
            return a + ab * distance;
        }
    }

    bool AreTwoFeaturePointsShared(Voxel _a, Voxel _b)
    {
        int sharedPoints = 0;

        foreach (Vector3 pointA in _a.master_featurePoints)
        {
            foreach (Vector3 pointB in _b.master_featurePoints)
            {
                if (pointA == pointB)
                {
                    sharedPoints++;

                    if (sharedPoints > 2) // If more than two shared points, return false
                        return false;
                }
            }
        }

        // Return true if exactly two points are shared, false otherwise
        return sharedPoints == 2;
    }

    List<Vector3> GetCubeCorners(GameObject cube)
    {
        Vector3 center = cube.transform.position;
        Vector3 scale = cube.transform.localScale / 2; 

        List<Vector3> corners = new List<Vector3>
        {
            center + new Vector3(-scale.x, -scale.y, -scale.z), // Bottom Back Left
            center + new Vector3(scale.x, -scale.y, -scale.z),  // Bottom Back Right
            center + new Vector3(-scale.x, -scale.y, scale.z),  // Bottom Front Left
            center + new Vector3(scale.x, -scale.y, scale.z),   // Bottom Front Right
            center + new Vector3(-scale.x, scale.y, -scale.z),  // Top Back Left
            center + new Vector3(scale.x, scale.y, -scale.z),   // Top Back Right
            center + new Vector3(-scale.x, scale.y, scale.z),   // Top Front Left
            center + new Vector3(scale.x, scale.y, scale.z)     // Top Front Right
        };

        return corners;
    }

    // Purely random
    void CreateFetaurePoints(GameObject _target)
    {
        float posX = _target.transform.position.x;
        float posY = _target.transform.position.y;
        float posZ = _target.transform.position.z;
        float xHelper = _target.transform.localScale.x / 2;
        float yHelper = _target.transform.localScale.y / 2;
        float zHelper = _target.transform.localScale.z / 2;

        for (int i = 0; i < numOfFeaturePoints; i++)
        {
            UnityEngine.Random.seed = UnityEngine.Random.Range(-1000, 1000);
            Vector3 newPoint = new Vector3(UnityEngine.Random.Range(posX - xHelper, posX + xHelper),
                UnityEngine.Random.Range(posY - yHelper, posY + yHelper),
                UnityEngine.Random.Range(posZ - zHelper, posZ + zHelper));
            featurePoints.Add(newPoint);
            //print(newPoint);
        }


    }
}
