using UnityEngine;
using System.Collections.Generic;

public class MeshHull : MonoBehaviour {

    private Mesh mesh;
    private static Vector3[] verts;
    private static List<Vector2> polygon;
    private Collider2D hullCollider2D;

    private static int CompareVerts(Vector3 lhs, Vector3 rhs)
    {
        int ret = lhs.x.CompareTo(rhs.x);

        if (ret == 0)
            ret = lhs.y.CompareTo(rhs.y);

        return ret;
    }

    private static float cross(Vector2 O, Vector2 A, Vector3 B)
    {
	    return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }

    void Start()
    {
        BuildHullCollider();
    }

    void BuildConvexHull()
    {
        verts = mesh.vertices;
        int n = verts.Length, k = 0;

        polygon = new List<Vector2>(2 * n);
        System.Array.Sort(verts, CompareVerts);


        // Build lower hull
        for (int i = 0; i < n; ++i)
        {
            while (k >= 2 && cross(polygon[k - 2], polygon[k - 1], verts[i]) <= 0.0) k--;
            if (polygon.Count == k)
                polygon.Add(verts[i]);
            else
                polygon[k] = verts[i];
            k++;
        }

        // Build upper hull
        for (int i = n - 2, t = k + 1; i >= 0; i--)
        {
            while (k >= t && cross(polygon[k - 2], polygon[k - 1], verts[i]) <= 0.0) k--;
            if (polygon.Count == k)
                polygon.Add(verts[i]);
            else
                polygon[k] = verts[i];
            k++;
        }

        polygon.TrimExcess();
    }

    public void BuildHullCollider()
    {
        mesh = GetComponent<MeshFilter>().sharedMesh;
        hullCollider2D = GetComponent<Collider2D>();

        BuildConvexHull();
        BuildConcaveHull();
        MergeEdgesType1();

       // SetCollider();
    }

    private int GetClosestToSegment(Vector2 a, Vector2 b)
    {
        Vector3 v = b - a;
        Vector3 n = Vector3.Cross(Vector3.forward, v);

        float delta = 0.000001f;

        float minSqrDist = float.MaxValue;
        int closest = -1;
        for (int i = 0; i < verts.Length; ++i)
        { 
            if (Vector2.Dot(((Vector2)n).normalized, ((Vector2)verts[i] - a).normalized) > -delta)
            {
                float sqrDist = SqDistPointSegment(a, b, verts[i]);
                if (sqrDist < minSqrDist)
                {
                    if (!polygon.Contains(verts[i]))
                    {
                        minSqrDist = sqrDist;
                        closest = i;
                    }
                }
            }
        }

        return closest;
    }

    private static float MinSum(Vector2 vert)
    {
        float min0 = float.MaxValue;
        float min1 = float.MaxValue;
        float min2 = float.MaxValue;
        float min3 = float.MaxValue;
        float min4 = float.MaxValue;
        for (int i = 0; i < verts.Length; ++i)
        {
            if (!polygon.Contains(verts[i]))
            {
                float sqrMag = ((Vector2)verts[i] - vert).magnitude;
                if (sqrMag < min0)
                {
                    min4 = min3;
                    min3 = min2;
                    min2 = min1;
                    min1 = min0;
                    min0 = sqrMag;
                }
                else if (sqrMag < min1)
                {
                    min4 = min3;
                    min3 = min2;
                    min2 = min1;
                    min1 = sqrMag;
                }
                else if (sqrMag < min2)
                {
                    min4 = min3;
                    min3 = min2;
                    min2 = sqrMag;
                }
                else if (sqrMag < min3)
                {
                    min4 = min3;
                    min3 = sqrMag;
                }
                else if (sqrMag < min4)
                    min4 = sqrMag;

            }
        }

        return min0 + min1 + min2 + min3 + min4;
    }

    private void BuildConcaveHull()
    {

        int i = 0;
        while (i < polygon.Count - 1)
        {
            Vector2 a = polygon[i];
            Vector2 b = polygon[i + 1];

            float absSqrLen = (b - a).magnitude;
            float d = 0.15f * (MinSum(a) + MinSum(b));

            if (absSqrLen >= d)
            {
                int c = GetClosestToSegment(a, b);
                if (c != -1)
                {
                    polygon.Insert(i + 1, verts[c]);
                }
                else
                    ++i;
            }
            else
                ++i;
        }

        polygon.RemoveAt(polygon.Count - 1);
        polygon.TrimExcess();

    }

    private void MergeEdgesType1()
    {
        int targetNumSides = 35;

        while (polygon.Count >= targetNumSides)
        {
            float minArea = float.MaxValue;
            int minVertIndex = 0;
            int a = 0;
            int b = 1;
            int c = 2;

            for (int count = 0; count < polygon.Count; ++count)
            {
                Vector2 v1 = (polygon[c] - polygon[b]).normalized;
                Vector2 v2 = (polygon[a] - polygon[b]).normalized;
                float angle = Mathf.Acos(Vector2.Dot(v1, v2));
                if(angle > 1.57079632679)
                {
                    float area = Vector3.Cross(v1, v2).magnitude;
                    if(area < minArea)
                    {
                        minArea = area;
                        minVertIndex = b;
                    }
                }
                a = (a + 1) % polygon.Count;
                b = (b + 1) % polygon.Count;
                c = (c + 1) % polygon.Count;
            }

            if (minArea != float.MaxValue)
                polygon.RemoveAt(minVertIndex);
            else
                break;

        }
    }

    private void SetCollider()
    {
        if(hullCollider2D.GetType() == typeof(PolygonCollider2D))
        {
            PolygonCollider2D polyCollider = hullCollider2D as PolygonCollider2D;
            polyCollider.pathCount = 1;
            polyCollider.SetPath(0, polygon.ToArray());
        }
        else if(hullCollider2D.GetType() == typeof(EdgeCollider2D))
        {
            polygon.Add(polygon[0]);
            (hullCollider2D as EdgeCollider2D).points = polygon.ToArray();
            polygon.RemoveAt(polygon.Count - 1);
        }
       
    }

    private void MergeEdgesType2()
    {
        int targetNumSides = 50;
        float rMax = 5.0f;
        float extend = float.MaxValue;

        while (polygon.Count >= targetNumSides)
        {
            float minArea = float.MaxValue;
            Vector2 newVert = Vector2.zero;
            int iMin1 = 0;
            int iMin2 = 0;
            int i0 = 0;
            int i1 = 1;
            int i2 = 2;
            int i3 = 3;

            for (int count = 0; count < polygon.Count; ++count)
            {

                Vector2 v1 = (polygon[i2] - polygon[i1]).normalized;
                Vector2 v2 = (polygon[i0] - polygon[i1]).normalized;

                Vector2 v3 = (polygon[i3] - polygon[i2]).normalized;
                Vector2 v4 = (polygon[i1] - polygon[i2]).normalized;


                float angle1 = Mathf.Acos(Vector2.Dot(v1, v2));
                float angle2 = Mathf.Acos(Vector2.Dot(v3, v4));

                if (angle1 > 1.57079632679 && angle2 > 1.57079632679)
                {
                    Vector2 dir1 = (polygon[i1] - polygon[i0]).normalized;
                    Vector2 dir2 = (polygon[i2] - polygon[i3]).normalized;
                    Vector2 res = Vector2.zero;

                    if (Test2DSegmentSegment(polygon[i0], polygon[i1] + extend * dir1, polygon[i3], polygon[i2] + extend * dir2, ref res))
                    {
                        Vector2 perp = ClosestPtPointSegment(res, polygon[i1], polygon[i2]);

                        if ((res - perp).magnitude / (polygon[i2] - polygon[i1]).magnitude < rMax)
                        {
                            Vector2 q = polygon[i2] - res;
                            Vector2 r = polygon[i1] - res;

                            float area = Vector3.Cross(q, r).magnitude;
                            if (area < minArea)
                            {
                                minArea = area;
                                iMin1 = i1;
                                iMin2 = i2;
                                newVert = res;
                            }
                        }
                    }
                }

                i0 = (i0 + 1) % polygon.Count;
                i1 = (i1 + 1) % polygon.Count;
                i2 = (i2 + 1) % polygon.Count;
                i3 = (i3 + 1) % polygon.Count;
            }

            if (minArea != float.MaxValue)
            {
                polygon[iMin1] = newVert;
                polygon.RemoveAt(iMin2);
            }
            else
                break;
        }
    }

    void OnDrawGizmos()
    {
        if (polygon == null)
            return;

        for (int i = 0; i < polygon.Count - 1; ++i)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(new Vector3(polygon[i].x, polygon[i].y, 0.0f), new Vector3(polygon[i + 1].x, polygon[i + 1].y, 0.0f));

            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(new Vector3(polygon[i].x, polygon[i].y, 0.0f), 0.05f);
        }

        Gizmos.color = Color.red;
        Gizmos.DrawLine(new Vector3(polygon[polygon.Count - 1].x, polygon[polygon.Count - 1].y, 0.0f), new Vector3(polygon[0].x, polygon[0].y, 0.0f));
        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(new Vector3(polygon[polygon.Count - 1].x, polygon[polygon.Count - 1].y, 0.0f), 0.05f);
    }

    //code example "from Real-Time Collision Detection by Christer Ericson, published by Morgan Kaufmaan Publishers, © 2005 Elvesier Inc"
    //----------------------------------------------------------------------------------------------------------------------------------
    // Returns the squared distance between point p and segment ab
    private static float SqDistPointSegment(Vector2 a, Vector2 b, Vector2 p)
    {
        Vector2 ab = b - a, ap = p - a, bp = p - b;
        float e = Vector2.Dot(ap, ab);
        if (e <= 0.0f)
            return Vector2.Dot(ap, ap);
        float f = Vector2.Dot(ab, ab);
        if (e >= f)
            return Vector2.Dot(bp, bp);
        return Vector2.Dot(ap, ap) - e * e / f;
    }

    private static float Signed2DTriArea(Vector2 a, Vector2 b, Vector2 c)
    {
        return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
    }

    bool Test2DSegmentSegment(Vector2 a, Vector2 b, Vector2 c, Vector2 d, ref Vector2 p)
    {
        // signs of areas correspond to which side of ab points c and d are
        float a1 = Signed2DTriArea(a, b, d); // Compute winding of abd (+ or -)
        float a2 = Signed2DTriArea(a, b, c); // To intersect, must have sign opposite of a1

        // If c and d are on different sides of ab, areas have different signs
        if (a1 * a2 < 0.0f) // require unsigned x & y values.
        {
            float a3 = Signed2DTriArea(c, d, a); // Compute winding of cda (+ or -)
            float a4 = a3 + a2 - a1; // Since area is constant a1 - a2 = a3 - a4, or a4 = a3 + a2 - a1

            // Points a and b on different sides of cd if areas have different signs
            if (a3 * a4 < 0.0f)
            {
                // Segments intersect. Find intersection point along L(t) = a + t * (b - a).
                float t = a3 / (a3 - a4);
                p = a + t * (b - a); // the point of intersection
                return true;
            }
        }

        // Segments not intersecting or collinear
        return false;
    }

    // Given segment ab and point c, computes closest point d on ab.
    // Also returns t for the position of d, d(t) = a + t*(b - a)
    Vector2 ClosestPtPointSegment(Vector2 c, Vector2 a, Vector2 b)
    {
        Vector2 ab = b - a;
        // Project c onto ab, computing parameterized position d(t) = a + t*(b – a)
        float t = Vector2.Dot(c - a, ab) / Vector2.Dot(ab, ab);
        // If outside segment, clamp t (and therefore d) to the closest endpoint
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;
        // Compute projected position from the clamped t
        return a + t * ab;
    }
    //----------------------------------------------------------------------------------------------------------------------------------



}
