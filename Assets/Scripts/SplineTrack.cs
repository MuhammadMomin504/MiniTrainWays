using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using System.Collections.Generic;

namespace Unity.Splines
{
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    [ExecuteInEditMode()]
    public class SplineTrack : MonoBehaviour
    {
        [SerializeField] private SplineSampler splineSampler = default;
        [SerializeField] private SplineContainer splineContainer;
        [SerializeField] private int splineIndex = 0;
        [SerializeField] private float m_Width = 0.5f;
        [SerializeField] private int resolution = 20;
        [SerializeField] private float m_TextureScale = 1.0f;
        [SerializeField] private Material m_JunctionMaterial;
        [SerializeField] private float m_JunctionSnapDistance = 0.5f;
        [SerializeField] private float m_JunctionInsetScale = 1f;
        [Tooltip("Max centerline distance to treat a drawn tip as colliding with an existing road.")]
        [SerializeField] private float m_ConnectDistance = -1f;

        private List<List<JunctionBuilder.JunctionInfo>> m_JunctionGroups = new List<List<JunctionBuilder.JunctionInfo>>();

        private MeshFilter m_MeshFilter;
        private Mesh m_Mesh;

        private List<Vector3> m_Vertices = new List<Vector3>();
        private List<int> m_RoadTriangles = new List<int>();
        private List<int> m_JunctionTriangles = new List<int>();
        private List<Vector2> m_UVs = new List<Vector2>();
        private List<Color> m_Colors = new List<Color>();

        private readonly Dictionary<int, float2> m_SplineTrimRanges = new Dictionary<int, float2>();

        public float JunctionSnapDistance => m_JunctionSnapDistance;
        public float RoadWidth => m_Width;

        public float ConnectDistance =>
            m_ConnectDistance > 0f ? m_ConnectDistance : math.max(m_JunctionSnapDistance, m_Width * 1.5f);

        private void Awake()
        {
            m_MeshFilter = GetComponent<MeshFilter>();
            if (m_MeshFilter.sharedMesh != null && m_MeshFilter.sharedMesh.name == "SplineMesh")
            {
                m_Mesh = m_MeshFilter.sharedMesh;
            }
            else
            {
                m_Mesh = new Mesh();
                m_Mesh.name = "SplineMesh";
                m_MeshFilter.mesh = m_Mesh;
            }

            m_Mesh.MarkDynamic();
        }

        /// <summary>
        /// Finds the nearest start/end knot among existing splines (optionally excluding one).
        /// </summary>
        public bool TrySnapToNearestEndpoint(float3 localPosition, int excludeSplineIndex, out float3 snappedLocal)
        {
            snappedLocal = localPosition;
            if (splineContainer == null || splineContainer.Splines == null)
                return false;

            float bestDist = m_JunctionSnapDistance;
            bool found = false;
            float3 best = localPosition;

            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                if (s == excludeSplineIndex) continue;

                var spline = splineContainer.Splines[s];
                if (spline == null || spline.Count < 1) continue;

                float3 start = spline[0].Position;
                float dStart = math.distance(localPosition, start);
                if (dStart < bestDist)
                {
                    bestDist = dStart;
                    best = start;
                    found = true;
                }

                if (spline.Count < 2) continue;

                float3 end = spline[spline.Count - 1].Position;
                float dEnd = math.distance(localPosition, end);
                if (dEnd < bestDist)
                {
                    bestDist = dEnd;
                    best = end;
                    found = true;
                }
            }

            if (found)
                snappedLocal = best;
            return found;
        }

        /// <summary>
        /// True when the connecting spline's drawing tip is within ConnectDistance of another road centerline.
        /// </summary>
        public bool WouldCollideWithExistingRoad(int connectingIndex)
        {
            return TryFindRoadCollision(connectingIndex, tipOnly: true, out _, out _, out _, out _, out _);
        }

        /// <summary>
        /// Snaps a drawn spline into the network: endpoint merge, or mid-road T-junction
        /// (insert knot, split hit road, snap tip, link knots).
        /// </summary>
        public bool ConnectSplineToExistingRoads(int connectingIndex)
        {
            if (splineContainer == null || connectingIndex < 0 || connectingIndex >= splineContainer.Splines.Count)
                return false;

            var connecting = splineContainer.Splines[connectingIndex];
            if (connecting == null || connecting.Count < 2)
                return false;

            // Prefer exact endpoint snaps first (tip, then start)
            bool connected = false;
            float3 startPos = connecting[0].Position;
            float3 endPos = connecting[connecting.Count - 1].Position;

            if (TrySnapToNearestEndpoint(endPos, connectingIndex, out float3 snapEnd))
            {
                TruncateCollapsedKnotsFromEnd(connecting, snapEnd);
                SetKnotPosition(connecting, connecting.Count - 1, snapEnd);
                connected = true;
            }

            if (TrySnapToNearestEndpoint(startPos, connectingIndex, out float3 snapStart))
            {
                TruncateCollapsedKnotsFromStart(connecting, snapStart);
                SetKnotPosition(connecting, 0, snapStart);
                connected = true;
            }

            if (connected)
                return true;

            // Mid-road / body collision from the stroke tip (drawing end)
            if (!TryFindRoadCollision(connectingIndex, tipOnly: true, out bool tipIsStart, out int hitSplineIndex,
                    out float hitT, out float3 hitPos, out _))
                return false;

            return FormJunctionAtHit(connectingIndex, tipIsStart, hitSplineIndex, hitT, hitPos);
        }

        private bool FormJunctionAtHit(
            int connectingIndex,
            bool tipIsStart,
            int hitSplineIndex,
            float hitT,
            float3 hitPos)
        {
            var connecting = splineContainer.Splines[connectingIndex];

            // Near an existing endpoint of the hit road — snap only
            if (hitT <= 0.05f || hitT >= 0.95f)
            {
                float3 endPoint = hitT <= 0.05f
                    ? splineContainer.Splines[hitSplineIndex][0].Position
                    : splineContainer.Splines[hitSplineIndex][splineContainer.Splines[hitSplineIndex].Count - 1].Position;

                if (tipIsStart)
                {
                    TruncateCollapsedKnotsFromStart(connecting, endPoint);
                    SetKnotPosition(connecting, 0, endPoint);
                }
                else
                {
                    TruncateCollapsedKnotsFromEnd(connecting, endPoint);
                    SetKnotPosition(connecting, connecting.Count - 1, endPoint);
                }

                return true;
            }

            // Mid-road hit → insert knot, split into two arms, snap connecting tip, link
            int knotIndex = InsertKnotAtNormalizedT(splineContainer.Splines[hitSplineIndex], hitT);
            if (knotIndex < 0)
                return false;

            var splitStart = SplineUtility.SplitSplineOnKnot(
                splineContainer,
                new SplineKnotIndex(hitSplineIndex, knotIndex));

            // After split: first half ends at junction; second half starts at junction
            var firstHalf = splineContainer.Splines[hitSplineIndex];
            var secondHalf = splineContainer.Splines[splitStart.Spline];

            // Pin all meeting tips to the exact same point for a seamless join
            SetKnotPosition(firstHalf, firstHalf.Count - 1, hitPos);
            SetKnotPosition(secondHalf, 0, hitPos);

            if (tipIsStart)
            {
                TruncateCollapsedKnotsFromStart(connecting, hitPos);
                SetKnotPosition(connecting, 0, hitPos);
            }
            else
            {
                TruncateCollapsedKnotsFromEnd(connecting, hitPos);
                SetKnotPosition(connecting, connecting.Count - 1, hitPos);
            }

            int tipKnot = tipIsStart ? 0 : connecting.Count - 1;
            SplineUtility.LinkKnots(splineContainer,
                new SplineKnotIndex(hitSplineIndex, firstHalf.Count - 1),
                new SplineKnotIndex(splitStart.Spline, 0));
            SplineUtility.LinkKnots(splineContainer,
                new SplineKnotIndex(hitSplineIndex, firstHalf.Count - 1),
                new SplineKnotIndex(connectingIndex, tipKnot));

            return true;
        }

        /// <summary>
        /// Finds the nearest existing road to the stroke tip (and optionally start).
        /// tipOnly=true is used while drawing so the body of a long stroke cannot false-trigger.
        /// </summary>
        private bool TryFindRoadCollision(
            int connectingIndex,
            bool tipOnly,
            out bool tipIsStart,
            out int hitSplineIndex,
            out float hitT,
            out float3 hitPos,
            out float connectingT)
        {
            tipIsStart = false;
            hitSplineIndex = -1;
            hitT = 0f;
            hitPos = float3.zero;
            connectingT = 1f;

            var connecting = splineContainer.Splines[connectingIndex];
            if (connecting == null || connecting.Count < 2)
                return false;

            float maxDist = ConnectDistance;
            float bestDist = float.MaxValue;
            bool found = false;

            // Drawing tip (end knot) — primary collision probe while sketching
            float3 endSample = connecting[connecting.Count - 1].Position;
            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                if (s == connectingIndex) continue;

                var other = splineContainer.Splines[s];
                if (other == null || other.Count < 2) continue;

                float dist = SplineUtility.GetNearestPoint(other, endSample, out float3 nearest, out float tHit);
                if (dist > maxDist || dist >= bestDist) continue;

                bestDist = dist;
                found = true;
                tipIsStart = false;
                hitSplineIndex = s;
                hitT = tHit;
                hitPos = nearest;
                connectingT = 1f;
            }

            if (!tipOnly)
            {
                float3 startSample = connecting[0].Position;
                for (int s = 0; s < splineContainer.Splines.Count; s++)
                {
                    if (s == connectingIndex) continue;

                    var other = splineContainer.Splines[s];
                    if (other == null || other.Count < 2) continue;

                    float dist = SplineUtility.GetNearestPoint(other, startSample, out float3 nearest, out float tHit);
                    if (dist > maxDist || dist >= bestDist) continue;

                    bestDist = dist;
                    found = true;
                    tipIsStart = true;
                    hitSplineIndex = s;
                    hitT = tHit;
                    hitPos = nearest;
                    connectingT = 0f;
                }
            }

            return found;
        }

        private static int InsertKnotAtNormalizedT(Spline spline, float t)
        {
            t = math.clamp(t, 0f, 1f);
            int curveIndex = spline.SplineToCurveT(t, out float curveT);

            if (curveT <= 0.02f)
                return curveIndex;
            if (curveT >= 0.98f)
                return math.min(curveIndex + 1, spline.Count - 1);

            // Mirror of Spline.InsertOnCurve using public APIs
            int insertAt = curveIndex + 1;
            var previous = spline[curveIndex];
            var next = spline[insertAt];

            var curveToSplit = new BezierCurve(previous, next);
            CurveUtility.Split(curveToSplit, curveT, out var leftCurve, out var rightCurve);

            if (spline.GetTangentMode(curveIndex) == TangentMode.Mirrored)
                spline.SetTangentMode(curveIndex, TangentMode.Continuous);
            if (spline.GetTangentMode(insertAt) == TangentMode.Mirrored)
                spline.SetTangentMode(insertAt, TangentMode.Continuous);

            TangentMode prevMode = spline.GetTangentMode(curveIndex);
            if (prevMode == TangentMode.Broken || prevMode == TangentMode.Continuous || prevMode == TangentMode.Mirrored)
            {
                previous.TangentOut = math.mul(math.inverse(previous.Rotation), leftCurve.Tangent0);
                spline.SetKnot(curveIndex, previous);
            }

            TangentMode nextMode = spline.GetTangentMode(insertAt);
            if (nextMode == TangentMode.Broken || nextMode == TangentMode.Continuous || nextMode == TangentMode.Mirrored)
            {
                next.TangentIn = math.mul(math.inverse(next.Rotation), rightCurve.Tangent1);
                spline.SetKnot(insertAt, next);
            }

            float3 up = math.up();
            var rotation = quaternion.LookRotationSafe(math.normalizesafe(rightCurve.Tangent0), up);
            var inverseRotation = math.inverse(rotation);
            var bezierKnot = new BezierKnot(
                leftCurve.P3,
                math.mul(inverseRotation, leftCurve.Tangent1),
                math.mul(inverseRotation, rightCurve.Tangent0),
                rotation);

            spline.Insert(insertAt, bezierKnot, TangentMode.Continuous);
            return insertAt;
        }

        private static void SetKnotPosition(Spline spline, int index, float3 position)
        {
            var knot = spline[index];
            knot.Position = position;
            spline.SetKnot(index, knot);
            spline.SetTangentMode(index, TangentMode.AutoSmooth);
        }

        private void TruncateCollapsedKnotsFromEnd(Spline spline, float3 hitPos)
        {
            float collapse = math.max(m_JunctionSnapDistance * 0.5f, 0.05f);
            while (spline.Count > 2)
            {
                float dLast = math.distance(spline[spline.Count - 1].Position, hitPos);
                float dPrev = math.distance(spline[spline.Count - 2].Position, hitPos);
                if (dPrev + 1e-4f < dLast || dPrev < collapse)
                    spline.RemoveAt(spline.Count - 1);
                else
                    break;
            }
        }

        private void TruncateCollapsedKnotsFromStart(Spline spline, float3 hitPos)
        {
            float collapse = math.max(m_JunctionSnapDistance * 0.5f, 0.05f);
            while (spline.Count > 2)
            {
                float d0 = math.distance(spline[0].Position, hitPos);
                float d1 = math.distance(spline[1].Position, hitPos);
                if (d1 + 1e-4f < d0 || d1 < collapse)
                    spline.RemoveAt(0);
                else
                    break;
            }
        }

        public void RebuildMesh()
        {
            if (splineContainer == null || splineContainer.Splines == null) return;

            if (m_MeshFilter == null)
                m_MeshFilter = GetComponent<MeshFilter>();
            if (m_Mesh == null)
            {
                if (m_MeshFilter.sharedMesh != null && m_MeshFilter.sharedMesh.name == "SplineMesh")
                    m_Mesh = m_MeshFilter.sharedMesh;
                else
                {
                    m_Mesh = new Mesh { name = "SplineMesh" };
                    m_MeshFilter.mesh = m_Mesh;
                    m_Mesh.MarkDynamic();
                }
            }

            m_Vertices.Clear();
            m_RoadTriangles.Clear();
            m_JunctionTriangles.Clear();
            m_UVs.Clear();
            m_Colors.Clear();

            BuildTrimRanges();

            Matrix4x4 worldToLocal = transform.worldToLocalMatrix;

            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                var spline = splineContainer.Splines[s];
                if (spline == null || spline.Count < 2) continue;

                float2 range = m_SplineTrimRanges.TryGetValue(s, out float2 r)
                    ? r
                    : new float2(0f, 1f);

                float tStart = range.x;
                float tEnd = range.y;
                if (tEnd <= tStart + 1e-4f)
                    continue;

                int vertexOffset = m_Vertices.Count;
                float splineLength = spline.GetLength();
                float sampledLength = math.max(0f, (tEnd - tStart) * splineLength);

                for (int i = 0; i <= resolution; i++)
                {
                    float t = math.lerp(tStart, tEnd, i / (float)resolution);

                    splineSampler.GetSplineWidthEdges(s, t, m_Width, out float3 leftEdge, out float3 rightEdge);

                    m_Vertices.Add(worldToLocal.MultiplyPoint3x4(leftEdge));
                    m_Vertices.Add(worldToLocal.MultiplyPoint3x4(rightEdge));

                    float v = ((t - tStart) / (tEnd - tStart)) * sampledLength * m_TextureScale;
                    m_UVs.Add(new Vector2(0, v));
                    m_UVs.Add(new Vector2(1, v));

                    m_Colors.Add(Color.white);
                    m_Colors.Add(Color.white);

                    if (i < resolution)
                    {
                        int start = vertexOffset + i * 2;
                        m_RoadTriangles.Add(start);
                        m_RoadTriangles.Add(start + 2);
                        m_RoadTriangles.Add(start + 1);

                        m_RoadTriangles.Add(start + 1);
                        m_RoadTriangles.Add(start + 2);
                        m_RoadTriangles.Add(start + 3);
                    }
                }
            }

            foreach (var junctionGroup in m_JunctionGroups)
            {
                // Junction shares the road submesh/material so it isn't a white patch
                BuildJunctionMesh(junctionGroup, m_Vertices, m_RoadTriangles, m_UVs, m_Colors);
            }

            if (m_Vertices.Count > 0)
            {
                m_Mesh.Clear();
                m_Mesh.SetVertices(m_Vertices);

                m_Mesh.subMeshCount = 1;
                m_Mesh.SetTriangles(m_RoadTriangles, 0);

                m_Mesh.SetUVs(0, m_UVs);
                m_Mesh.SetColors(m_Colors);
                m_Mesh.RecalculateNormals();
                m_Mesh.RecalculateBounds();
            }
            else
            {
                m_Mesh.Clear();
            }
        }

        private void BuildTrimRanges()
        {
            m_SplineTrimRanges.Clear();

            for (int s = 0; s < splineContainer.Splines.Count; s++)
                m_SplineTrimRanges[s] = new float2(0f, 1f);

            foreach (var group in m_JunctionGroups)
            {
                if (group == null) continue;
                foreach (var junction in group)
                {
                    if (!m_SplineTrimRanges.TryGetValue(junction.splineIndex, out float2 range))
                        continue;

                    var spline = splineContainer.Splines[junction.splineIndex];
                    if (spline == null || spline.Count < 2) continue;

                    float insetT = GetInsetT(spline);
                    if (junction.t <= 0.5f)
                        range.x = math.max(range.x, insetT);
                    else
                        range.y = math.min(range.y, 1f - insetT);

                    m_SplineTrimRanges[junction.splineIndex] = range;
                }
            }
        }

        private float GetInsetT(Spline spline)
        {
            float length = math.max(spline.GetLength(), 1e-4f);
            float insetDistance = m_Width * m_JunctionInsetScale;
            return math.clamp(insetDistance / length, 0.01f, 0.45f);
        }

        private float GetArmSampleT(JunctionBuilder.JunctionInfo junction)
        {
            var spline = splineContainer.Splines[junction.splineIndex];
            float insetT = GetInsetT(spline);
            return junction.t <= 0.5f ? insetT : 1f - insetT;
        }

        private void BuildJunctionMesh(
            List<JunctionBuilder.JunctionInfo> junctions,
            List<Vector3> vertices,
            List<int> triangles,
            List<Vector2> uvs,
            List<Color> colors)
        {
            if (junctions == null || junctions.Count < 2) return;
            if (splineSampler == null) return;

            Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
            var corners = new List<Vector3>(junctions.Count * 2);
            Vector3 centerAccum = Vector3.zero;

            foreach (var junction in junctions)
            {
                if (junction.splineIndex < 0 || junction.splineIndex >= splineContainer.Splines.Count)
                    continue;

                // Sample the exact same edge points the road strip uses at its trim boundary
                // so the junction fan meets the roads with no gap or overlap seam.
                float sampleT = GetArmSampleT(junction);
                splineSampler.GetSplineWidthEdges(
                    junction.splineIndex, sampleT, m_Width,
                    out float3 leftW, out float3 rightW);

                Vector3 leftCorner = worldToLocal.MultiplyPoint3x4(leftW);
                Vector3 rightCorner = worldToLocal.MultiplyPoint3x4(rightW);

                corners.Add(rightCorner);
                corners.Add(leftCorner);
                centerAccum += rightCorner + leftCorner;
            }

            if (corners.Count < 4) return;

            Vector3 center = centerAccum / corners.Count;

            // Sort every corner around the center so the fan cannot self-intersect / spike
            corners.Sort((a, b) =>
            {
                float angleA = Mathf.Atan2(a.z - center.z, a.x - center.x);
                float angleB = Mathf.Atan2(b.z - center.z, b.x - center.x);
                return angleA.CompareTo(angleB);
            });

            int centerIndex = vertices.Count;
            vertices.Add(center);
            uvs.Add(new Vector2(center.x * m_TextureScale, center.z * m_TextureScale));
            colors.Add(Color.white);

            int first = vertices.Count;
            for (int i = 0; i < corners.Count; i++)
            {
                Vector3 c = corners[i];
                vertices.Add(c);
                uvs.Add(new Vector2(c.x * m_TextureScale, c.z * m_TextureScale));
                colors.Add(Color.white);
            }

            for (int i = 0; i < corners.Count; i++)
            {
                int current = first + i;
                int next = first + (i + 1) % corners.Count;

                triangles.Add(centerIndex);
                triangles.Add(next);
                triangles.Add(current);
            }
        }

        public void RemoveJunction(JunctionBuilder.JunctionInfo junction)
        {
            foreach (var group in m_JunctionGroups)
            {
                if (group.Remove(junction)) break;
            }
        }

        public void ClearJunctions()
        {
            m_JunctionGroups.Clear();
            RebuildMesh();
        }

        public void AddJunction(JunctionBuilder builder)
        {
            if (builder.junctions != null && builder.junctions.Count > 0)
            {
                m_JunctionGroups.Add(new List<JunctionBuilder.JunctionInfo>(builder.junctions));
            }
            RebuildMesh();
        }

        [ContextMenu("Auto Detect Junctions")]
        public void AutoDetectJunctions()
        {
            if (splineContainer == null || splineContainer.Splines == null) return;

            m_JunctionGroups.Clear();

            var candidates = new List<JunctionBuilder.JunctionInfo>();

            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                var spline = splineContainer.Splines[s];
                if (spline == null || spline.Count < 2) continue;

                candidates.Add(new JunctionBuilder.JunctionInfo(s, 0, 0f, spline, spline[0]));
                int last = spline.Count - 1;
                candidates.Add(new JunctionBuilder.JunctionInfo(s, last, 1f, spline, spline[last]));
            }

            int n = candidates.Count;
            int[] parent = new int[n];
            for (int i = 0; i < n; i++)
                parent[i] = i;

            int Find(int i)
            {
                while (parent[i] != i)
                {
                    parent[i] = parent[parent[i]];
                    i = parent[i];
                }
                return i;
            }

            void Union(int a, int b)
            {
                int ra = Find(a);
                int rb = Find(b);
                if (ra != rb)
                    parent[rb] = ra;
            }

            float snapSq = m_JunctionSnapDistance * m_JunctionSnapDistance;
            for (int i = 0; i < n; i++)
            {
                float3 pi = candidates[i].knot.Position;
                for (int j = i + 1; j < n; j++)
                {
                    if (candidates[i].splineIndex == candidates[j].splineIndex)
                        continue;

                    if (math.distancesq(pi, candidates[j].knot.Position) <= snapSq)
                        Union(i, j);
                }
            }

            var clusters = new Dictionary<int, List<JunctionBuilder.JunctionInfo>>();
            for (int i = 0; i < n; i++)
            {
                int root = Find(i);
                if (!clusters.TryGetValue(root, out var list))
                {
                    list = new List<JunctionBuilder.JunctionInfo>();
                    clusters[root] = list;
                }
                list.Add(candidates[i]);
            }

            foreach (var group in clusters.Values)
            {
                if (group.Count > 1)
                    m_JunctionGroups.Add(group);
            }

            RebuildMesh();
        }
    }
}
