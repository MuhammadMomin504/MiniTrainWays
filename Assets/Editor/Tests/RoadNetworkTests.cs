using System.Collections.Generic;
using System.IO;
using System.Reflection;
using NUnit.Framework;
using Unity.Mathematics;
using Unity.Splines;
using UnityEditor;
using UnityEngine;
using UnityEngine.Splines;

/// <summary>
/// Builds road networks through the same SplineTrack API RuntimeSplineGenerator uses and inspects
/// the generated mesh: full coverage (no gaps), single coverage (no overlapping surfaces), upward
/// normals and winding, UVs that follow each road without stretching, and junction pads that
/// carry no lane markings.
/// </summary>
public class RoadNetworkTests
{
    const float DefaultWidth = 0.2f;
    const float KnotSpacing = 0.1f;
    const BindingFlags Private = BindingFlags.NonPublic | BindingFlags.Instance;

    GameObject m_Root;
    SplineContainer m_Container;
    SplineTrack m_Track;

    [SetUp]
    public void SetUp()
    {
        m_Root = new GameObject("RoadNetworkTest");
        m_Root.AddComponent<MeshFilter>();
        m_Root.AddComponent<MeshRenderer>();
        m_Container = m_Root.AddComponent<SplineContainer>();
        while (m_Container.Splines.Count > 0)
            m_Container.RemoveSplineAt(0);

        var sampler = m_Root.AddComponent<SplineSampler>();
        SetField(sampler, "splineContainer", m_Container);

        m_Track = m_Root.AddComponent<SplineTrack>();
        SetField(m_Track, "splineSampler", sampler);
        SetField(m_Track, "splineContainer", m_Container);
        SetField(m_Track, "m_Width", DefaultWidth);
        SetField(m_Track, "resolution", 200);
        SetField(m_Track, "m_JunctionSnapDistance", 0.5f);
        SetField(m_Track, "m_TextureScale", 1f);
    }

    [TearDown]
    public void TearDown()
    {
        Object.DestroyImmediate(m_Root);
    }

    [Test]
    public void StraightAndCurvedRoads_UVsFollowRoadWithoutStretching()
    {
        AddRoad(new Vector3(-3f, 0f, 0f), new Vector3(3f, 0f, 0f));
        int curved = AddRoad(Arc(new Vector3(0f, 0f, 3f), 2f, 200f, 340f, 24));
        m_Track.SetRoadWidth(m_Container.Splines[curved], 0.35f);
        m_Track.RebuildMesh();

        AssertCleanMesh(expectedPads: 0);
        Assert.AreEqual(2, RoadVertexCount() / StripVertexCount());
        Assert.AreEqual(Mesh.vertexCount, RoadVertexCount(), "no junction geometry on unconnected roads");
    }

    [Test]
    public void TJunction_FormsOnePadWithCleanSeams()
    {
        AddRoad(new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 6f));
        Connect(AddRoad(new Vector3(-3f, 0f, 3f), new Vector3(0f, 0f, 3f)));

        Assert.AreEqual(3, m_Container.Splines.Count, "through road should be split at the junction");
        AssertCleanMesh(expectedPads: 1);
        Assert.AreEqual(3, PadArmCount(0));
    }

    [Test]
    public void CrossIntersection_JoinsFourArmsInOnePad()
    {
        AddRoad(new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 6f));
        Connect(AddRoad(new Vector3(-3f, 0f, 3f), new Vector3(0f, 0f, 3f)));
        Connect(AddRoad(new Vector3(3f, 0f, 3f), new Vector3(0f, 0f, 3f)));

        AssertCleanMesh(expectedPads: 1);
        Assert.AreEqual(4, PadArmCount(0));
    }

    [Test]
    public void AngledJunctions_StayClean()
    {
        AddRoad(new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 6f));
        Connect(AddRoad(new Vector3(-3f, 0f, 1f), new Vector3(0f, 0f, 4f)));                  // 45 degrees
        Connect(AddRoad(new Vector3(3f, 0f, 0.6f), new Vector3(0f, 0f, 0.6f + 3f * 0.577f))); // 30 degrees

        AssertCleanMesh(expectedPads: 2);
    }

    [Test]
    public void DifferentWidths_ConnectCleanlyAndKeepEachWidth()
    {
        int main = AddRoad(new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 6f));
        m_Track.SetRoadWidth(m_Container.Splines[main], 0.3f);
        int wide = AddRoad(new Vector3(-3f, 0f, 2f), new Vector3(0f, 0f, 2f));
        m_Track.SetRoadWidth(m_Container.Splines[wide], 0.4f);
        Connect(wide);
        int narrow = AddRoad(new Vector3(3f, 0f, 4.2f), new Vector3(0f, 0f, 4.2f));
        m_Track.SetRoadWidth(m_Container.Splines[narrow], 0.12f);
        Connect(narrow);

        AssertCleanMesh(expectedPads: 2);

        int mainPieces = 0;
        for (int s = 0; s < m_Container.Splines.Count; s++)
        {
            var spline = m_Container.Splines[s];
            if (math.abs(spline[0].Position.x) < 1e-3f && math.abs(spline[spline.Count - 1].Position.x) < 1e-3f)
            {
                mainPieces++;
                Assert.AreEqual(0.3f, m_Track.GetRoadWidth(spline), 1e-5f, "split pieces must keep the road's width");
            }
        }
        Assert.AreEqual(3, mainPieces);
    }

    [Test]
    public void BendsAndWidthChangesBetweenTwoRoads_GetAPadWithoutGaps()
    {
        AddRoad(new Vector3(-3f, 0f, 1f), new Vector3(0f, 0f, 1f));
        Connect(AddRoad(new Vector3(0f, 0f, 1f), new Vector3(1.6f, 0f, 2.6f)));   // 135 degree bend

        int a = AddRoad(new Vector3(-3f, 0f, 4f), new Vector3(0f, 0f, 4f));
        m_Track.SetRoadWidth(m_Container.Splines[a], 0.3f);
        Connect(AddRoad(new Vector3(0f, 0f, 4f), new Vector3(3f, 0f, 4f)));       // straight, narrower

        AssertCleanMesh(expectedPads: 2);

        // Both are lofted as marked strips, so lane paint carries through the bend and the taper.
        var colors = Mesh.colors;
        for (int i = RoadVertexCount(); i < colors.Length; i++)
            Assert.Greater(colors[i].a, 0.99f, "two-road bends keep their lane markings");
    }

    [Test]
    public void EditingAndDisconnecting_RebuildsJunctions()
    {
        AddRoad(new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 6f));
        int side = AddRoad(new Vector3(-3f, 0f, 3f), new Vector3(0f, 0f, 3f));
        Connect(side);
        Assert.AreEqual(1, PadCount());

        int sideIndex = IndexOfSideRoad();
        var road = m_Container.Splines[sideIndex];
        int tip = road.Count - 1;

        // Editing: pull the side road's far end out; the mesh and junction rebuild on the next frame.
        var far = road[0];
        far.Position = new float3(-3.5f, 0f, 3f);
        road.SetKnot(0, far);
        SimulateFrame();
        Assert.Less(Mesh.bounds.min.x, -3.45f, "the edited road must be rebuilt");
        AssertCleanMesh(expectedPads: 1);
        Assert.AreEqual(3, PadArmCount(0));

        // Disconnecting: unlink the side road and move it away from the junction.
        m_Container.KnotLinkCollection.Unlink(new SplineKnotIndex(sideIndex, tip));
        TranslateRoad(road, new float3(-1.2f, 0f, 0f));
        SimulateFrame();
        AssertCleanMesh(expectedPads: 0);

        // Reconnecting: move it back onto the junction point.
        TranslateRoad(road, new float3(1.2f, 0f, 0f));
        SimulateFrame();
        AssertCleanMesh(expectedPads: 1);
        Assert.AreEqual(3, PadArmCount(0));
    }

    [Test]
    public void CornerTighterThanRoad_DoesNotFoldTheStrip()
    {
        // Knots every 0.1 around a right angle give a corner radius well under the 0.2 half-width.
        AddRoad(new Vector3(-2f, 0f, 0f), new Vector3(0f, 0f, 0f), new Vector3(0f, 0f, 2f));
        m_Track.RebuildMesh();

        // The inside of such a corner cannot be paved at full width, but it must never fold over.
        AssertCleanMesh(expectedPads: 0, fullWidthPaved: false);
    }

    [Test]
    public void RoadSurfaceAssets_TileAndKeepJunctionsFreeOfMarkings()
    {
        var material = AssetDatabase.LoadAssetAtPath<Material>(RoadSurfaceGenerator.MaterialPath);
        Assert.IsNotNull(material, "run Tools/MiniTrainWays/Generate Road Surface");
        Assert.AreEqual("MiniTrainWays/Road Surface", material.shader.name);
        Assert.IsTrue(material.shader.isSupported);
        Assert.IsNotNull(material.GetTexture("_AsphaltTex"));
        Assert.IsNotNull(material.GetTexture("_MarkingsTex"));

        var asphalt = LoadPng(RoadSurfaceGenerator.AsphaltPath);
        float interior = 0f, wrapX = 0f, wrapY = 0f;
        int size = asphalt.width;
        for (int i = 0; i < size; i++)
        {
            interior += Diff(asphalt.GetPixel(size / 2, i), asphalt.GetPixel(size / 2 + 1, i));
            wrapX += Diff(asphalt.GetPixel(size - 1, i), asphalt.GetPixel(0, i));
            wrapY += Diff(asphalt.GetPixel(i, size - 1), asphalt.GetPixel(i, 0));
        }
        Assert.Less(wrapX, interior * 1.5f, "asphalt must tile seamlessly across U");
        Assert.Less(wrapY, interior * 1.5f, "asphalt must tile seamlessly across V");

        var markings = LoadPng(RoadSurfaceGenerator.MarkingsPath);
        for (int y = 0; y < markings.height; y += 8)
        {
            Assert.Less(markings.GetPixel(Mathf.RoundToInt(0.3f * markings.width), y).a, 0.01f, "lanes must be unpainted");
            Assert.Greater(markings.GetPixel(Mathf.RoundToInt(0.085f * markings.width), y).a, 0.5f, "edge line must be solid");
        }
        Object.DestroyImmediate(asphalt);
        Object.DestroyImmediate(markings);
    }

    // ---------------------------------------------------------------- network helpers

    int AddRoad(params Vector3[] points)
    {
        var spline = m_Container.AddSpline();
        for (int p = 1; p < points.Length; p++)
        {
            float length = Vector3.Distance(points[p - 1], points[p]);
            int steps = Mathf.Max(1, Mathf.RoundToInt(length / KnotSpacing));
            for (int k = p == 1 ? 0 : 1; k <= steps; k++)
                spline.Add(new BezierKnot((float3)Vector3.Lerp(points[p - 1], points[p], k / (float)steps)), TangentMode.AutoSmooth);
        }
        return m_Container.Splines.Count - 1;
    }

    static Vector3[] Arc(Vector3 center, float radius, float fromDeg, float toDeg, int segments)
    {
        var points = new Vector3[segments + 1];
        for (int i = 0; i <= segments; i++)
        {
            float a = Mathf.Lerp(fromDeg, toDeg, i / (float)segments) * Mathf.Deg2Rad;
            points[i] = center + new Vector3(Mathf.Cos(a), 0f, Mathf.Sin(a)) * radius;
        }
        return points;
    }

    void Connect(int splineIndex)
    {
        Assert.IsTrue(m_Track.ConnectSplineToExistingRoads(splineIndex), "road should connect");
        m_Track.AutoDetectJunctions();
    }

    static void TranslateRoad(Spline road, float3 delta)
    {
        for (int k = 0; k < road.Count; k++)
        {
            var knot = road[k];
            knot.Position += delta;
            road.SetKnot(k, knot);
        }
    }

    void SimulateFrame()
    {
        typeof(SplineTrack).GetMethod("LateUpdate", Private).Invoke(m_Track, null);
    }

    int IndexOfSideRoad()
    {
        for (int s = 0; s < m_Container.Splines.Count; s++)
            if (math.abs(m_Container.Splines[s][0].Position.x + 3f) < 1e-3f)
                return s;
        Assert.Fail("side road not found");
        return -1;
    }

    int PadCount() => ((System.Collections.IList)typeof(SplineTrack).GetField("m_JunctionPads", Private).GetValue(m_Track)).Count;

    int PadArmCount(int pad)
    {
        var groups = (System.Collections.IList)typeof(SplineTrack).GetField("m_JunctionGroups", Private).GetValue(m_Track);
        int found = 0;
        foreach (System.Collections.IList group in groups)
            if (found++ == pad)
                return group.Count;
        return 0;
    }

    int StripVertexCount() => ((int)typeof(SplineTrack).GetField("resolution", Private).GetValue(m_Track) + 1) * 2;

    // Road strips come first in the mesh, one per spline piece left after junction trimming.
    int RoadVertexCount() => StripSplines().Count * StripVertexCount();

    List<int> StripSplines()
    {
        var stripSplines = new List<int>();
        for (int s = 0; s < m_Container.Splines.Count; s++)
        {
            float2 range = TrimRange(s);
            if (m_Container.Splines[s].Count >= 2 && range.y > range.x + 1e-4f)
                stripSplines.Add(s);
        }
        return stripSplines;
    }

    Mesh Mesh => m_Root.GetComponent<MeshFilter>().sharedMesh;

    // ---------------------------------------------------------------- mesh inspection

    void AssertCleanMesh(int expectedPads, bool fullWidthPaved = true)
    {
        Assert.AreEqual(expectedPads, PadCount(), "junction pad count");

        var mesh = Mesh;
        var vertices = mesh.vertices;
        var normals = mesh.normals;
        var uvs = ProjectedUVs(mesh);
        var colors = mesh.colors;
        var triangles = mesh.triangles;

        for (int i = 0; i < normals.Length; i++)
            Assert.Greater(normals[i].y, 0.999f, $"normal {i} must point up");

        // Zero-area triangles are allowed (an inner edge held still around a tight bend); flipped ones are not.
        int degenerate = 0;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            var n = Vector3.Cross(vertices[triangles[i + 1]] - vertices[triangles[i]], vertices[triangles[i + 2]] - vertices[triangles[i]]);
            Assert.Greater(n.y, -1e-9f, $"triangle {i / 3} is flipped");
            if (n.y <= 1e-9f) degenerate++;
        }
        Assert.Less(degenerate, triangles.Length / 3 / 20, "too many collapsed triangles");

        AssertRoadUVs(vertices, uvs, colors);
        AssertPadUVs(vertices, uvs, colors);
        AssertSingleCoverage(vertices, triangles, fullWidthPaved);
    }

    void AssertRoadUVs(Vector3[] vertices, Vector2[] uvs, Color[] colors)
    {
        int roadVertices = RoadVertexCount();
        int strip = StripVertexCount();
        int resolution = strip / 2 - 1;
        Assert.AreEqual(0, roadVertices % strip, "road vertices come in whole strips");

        var stripSplines = StripSplines();
        for (int i = 0; i < roadVertices; i++)
            Assert.Greater(colors[i].a, 0.99f, "road strips carry lane markings");

        for (int k = 0, start = 0; start < roadVertices; k++, start += strip)
        {
            var spline = m_Container.Splines[stripSplines[k]];
            float2 range = TrimRange(stripSplines[k]);
            float halfWidth = m_Track.GetRoadWidth(spline);
            Assert.AreEqual(halfWidth * 2f, Vector3.Distance(vertices[start], vertices[start + 1]), 1e-4f, "road width at its start");
            float expectedRate = 1f / (2f * halfWidth);

            float distance = 0f;
            Vector3 previous = Vector3.zero;
            for (int i = 0; i < strip; i += 2)
            {
                Assert.AreEqual(0f, uvs[start + i].x, 1e-6f, "left edge u");
                Assert.AreEqual(1f, uvs[start + i + 1].x, 1e-6f, "right edge u");
                Assert.LessOrEqual(Vector3.Distance(vertices[start + i], vertices[start + i + 1]), halfWidth * 2f + 1e-4f, "road never widens");

                Vector3 center = (Vector3)SplineUtility.EvaluatePosition(spline, math.lerp(range.x, range.y, i / 2 / (float)resolution));
                if (i > 0)
                    distance += Vector3.Distance(center, previous);
                previous = center;
            }

            // V advances at one unit per road width along the real centerline: no stretching.
            float vSpan = uvs[start + strip - 2].y - uvs[start].y;
            Assert.AreEqual(distance * expectedRate, vSpan, distance * expectedRate * 0.01f + 1e-4f, "V must follow road length");
            for (int i = 2; i < strip; i += 2)
                Assert.GreaterOrEqual(uvs[start + i].y, uvs[start + i - 2].y, "V must run one way along the road");
        }
    }

    void AssertPadUVs(Vector3[] vertices, Vector2[] uvs, Color[] colors)
    {
        for (int i = RoadVertexCount(); i < vertices.Length; i++)
        {
            if (colors[i].a > 0.99f)
            {
                // Two-road bend lofted as a marked strip: road-style UVs.
                Assert.IsTrue(Mathf.Approximately(uvs[i].x, 0f) || Mathf.Approximately(uvs[i].x, 1f), "bend strip u");
                Assert.GreaterOrEqual(uvs[i].y, 0f, "bend strip v");
                continue;
            }

            // Intersection pad: no lane markings, planar UVs.
            Assert.Less(colors[i].a, 0.01f, "junction pads must not carry lane markings");
            Vector2 planar = new Vector2(vertices[i].x, vertices[i].z);
            if (planar.sqrMagnitude < 1e-6f) continue;
            float scale = uvs[i].magnitude / planar.magnitude;
            Assert.AreEqual(0f, Vector2.Angle(planar, uvs[i]), 0.1f, "pad UVs must be planar (no smearing)");
            Assert.Greater(scale, 0f);
        }
    }

    void AssertSingleCoverage(Vector3[] vertices, int[] triangles, bool fullWidthPaved)
    {
        var grid = new TriangleGrid(vertices, triangles, 0.1f);

        // No overlapping surfaces: every sample is strictly inside at most one triangle.
        Bounds bounds = Mesh.bounds;
        const float step = 0.0123f;
        for (float x = bounds.min.x; x <= bounds.max.x; x += step)
        for (float z = bounds.min.z; z <= bounds.max.z; z += step)
        {
            int hits = grid.Count(new Vector2(x, z), strict: true);
            Assert.LessOrEqual(hits, 1, $"overlapping road surfaces at ({x:F3}, {z:F3})");
        }

        // No gaps: every road's full width is paved along its strip, its centerline is paved right
        // into the junction, and just past each strip end the junction covers the full road width.
        for (int s = 0; s < m_Container.Splines.Count; s++)
        {
            var spline = m_Container.Splines[s];
            if (spline.Count < 2) continue;
            float halfWidth = m_Track.GetRoadWidth(spline);
            float length = spline.GetLength();
            float2 range = TrimRange(s);
            int samples = Mathf.CeilToInt(length / 0.01f);
            for (int i = 0; i <= samples; i++)
            {
                float t = Mathf.Clamp(i / (float)samples, 1e-4f, 1f - 1e-4f);
                bool onStrip = t >= range.x && t <= range.y;
                AssertPaved(grid, spline, s, t, halfWidth, onStrip && fullWidthPaved ? new[] { 0f, 0.9f, -0.9f } : new[] { 0f });
            }

            float pastEnd = 0.005f / length;
            if (range.x > 0f)
                AssertPaved(grid, spline, s, range.x - pastEnd, halfWidth, new[] { 0f, 0.5f, -0.5f, 0.9f, -0.9f });
            if (range.y < 1f)
                AssertPaved(grid, spline, s, range.y + pastEnd, halfWidth, new[] { 0f, 0.5f, -0.5f, 0.9f, -0.9f });
        }
    }

    static void AssertPaved(TriangleGrid grid, Spline spline, int index, float t, float halfWidth, float[] offsets)
    {
        SplineUtility.Evaluate(spline, t, out float3 position, out float3 tangent, out _);
        float3 side = math.normalizesafe(math.cross(math.normalizesafe(tangent), new float3(0f, 1f, 0f)));
        foreach (float offset in offsets)
        {
            float3 p = position + side * halfWidth * offset;
            Assert.GreaterOrEqual(grid.Count(new Vector2(p.x, p.z), strict: false), 1,
                $"gap in road {index} at t={t:F3} offset {offset} ({p.x:F3}, {p.z:F3})");
        }
    }

    float2 TrimRange(int splineIndex)
    {
        var ranges = (Dictionary<int, float2>)typeof(SplineTrack).GetField("m_SplineTrimRanges", Private).GetValue(m_Track);
        return ranges.TryGetValue(splineIndex, out float2 range) ? range : new float2(0f, 1f);
    }

    sealed class TriangleGrid
    {
        readonly Vector3[] m_Vertices;
        readonly int[] m_Triangles;
        readonly float m_Cell;
        readonly Dictionary<long, List<int>> m_Cells = new Dictionary<long, List<int>>();

        public TriangleGrid(Vector3[] vertices, int[] triangles, float cell)
        {
            m_Vertices = vertices;
            m_Triangles = triangles;
            m_Cell = cell;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 a = vertices[triangles[i]], b = vertices[triangles[i + 1]], c = vertices[triangles[i + 2]];
                int x0 = Mathf.FloorToInt(Mathf.Min(a.x, Mathf.Min(b.x, c.x)) / cell);
                int x1 = Mathf.FloorToInt(Mathf.Max(a.x, Mathf.Max(b.x, c.x)) / cell);
                int z0 = Mathf.FloorToInt(Mathf.Min(a.z, Mathf.Min(b.z, c.z)) / cell);
                int z1 = Mathf.FloorToInt(Mathf.Max(a.z, Mathf.Max(b.z, c.z)) / cell);
                for (int x = x0; x <= x1; x++)
                for (int z = z0; z <= z1; z++)
                {
                    long key = ((long)x << 32) ^ (uint)z;
                    if (!m_Cells.TryGetValue(key, out var list))
                        m_Cells[key] = list = new List<int>();
                    list.Add(i);
                }
            }
        }

        public int Count(Vector2 p, bool strict)
        {
            long key = ((long)Mathf.FloorToInt(p.x / m_Cell) << 32) ^ (uint)Mathf.FloorToInt(p.y / m_Cell);
            if (!m_Cells.TryGetValue(key, out var list)) return 0;

            int hits = 0;
            foreach (int i in list)
            {
                Vector3 a = m_Vertices[m_Triangles[i]], b = m_Vertices[m_Triangles[i + 1]], c = m_Vertices[m_Triangles[i + 2]];
                float area = Edge(a, b, c.x, c.z);
                if (Mathf.Abs(area) < 1e-12f) continue;
                float sign = Mathf.Sign(area);
                float w0 = Edge(b, c, p.x, p.y) * sign;
                float w1 = Edge(c, a, p.x, p.y) * sign;
                float w2 = Edge(a, b, p.x, p.y) * sign;
                bool inside = strict
                    ? w0 > 1e-6f && w1 > 1e-6f && w2 > 1e-6f
                    : w0 >= -1e-6f && w1 >= -1e-6f && w2 >= -1e-6f;
                if (inside) hits++;
            }
            return hits;
        }

        static float Edge(Vector3 a, Vector3 b, float px, float pz) => (b.x - a.x) * (pz - a.z) - (b.z - a.z) * (px - a.x);
    }

    // ---------------------------------------------------------------- misc

    // Road UVs are stored projectively as (u·q, v·q, q); the shader divides by q, so do the same here.
    static Vector2[] ProjectedUVs(Mesh mesh)
    {
        var uvq = new List<Vector3>();
        mesh.GetUVs(0, uvq);
        var uvs = new Vector2[uvq.Count];
        for (int i = 0; i < uvq.Count; i++)
        {
            Assert.Greater(uvq[i].z, 0f, "UV q must be positive");
            uvs[i] = new Vector2(uvq[i].x, uvq[i].y) / uvq[i].z;
        }
        return uvs;
    }

    static void SetField(object target, string name, object value)
    {
        var field = target.GetType().GetField(name, Private);
        Assert.IsNotNull(field, name);
        field.SetValue(target, value);
    }

    static Texture2D LoadPng(string path)
    {
        var texture = new Texture2D(2, 2);
        texture.LoadImage(File.ReadAllBytes(path));
        return texture;
    }

    static float Diff(Color a, Color b) => Mathf.Abs(a.r - b.r) + Mathf.Abs(a.g - b.g) + Mathf.Abs(a.b - b.b);
}
