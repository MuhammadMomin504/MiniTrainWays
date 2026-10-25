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
        [Tooltip("Default road half-width. Each spline can override it (see SetRoadWidth).")]
        [SerializeField] private float m_Width = 0.5f;
        [SerializeField] private int resolution = 20;
        [Tooltip("Along-road UV per road width. 1 keeps lane markings in proportion to the road width.")]
        [SerializeField] private float m_TextureScale = 1.0f;
        [SerializeField] private Material m_JunctionMaterial;
        [SerializeField] private float m_JunctionSnapDistance = 0.5f;
        [Tooltip("Extra road length (in road widths) given to the curved blend between roads at a junction.")]
        [SerializeField] private float m_JunctionInsetScale = 1f;
        [Tooltip("Lerp subdivisions used for each curved edge between two junction arms.")]
        [SerializeField] [Min(1)] private int m_JunctionCurveSegments = 8;
        [Tooltip("Max centerline distance to treat a drawn tip as colliding with an existing road.")]
        [SerializeField] private float m_ConnectDistance = -1f;

        private const string k_RoadWidthKey = "roadWidth";

        private List<List<JunctionBuilder.JunctionInfo>> m_JunctionGroups = new List<List<JunctionBuilder.JunctionInfo>>();
        private bool m_NetworkDirty;

        private MeshFilter m_MeshFilter;
        private Mesh m_Mesh;

        private List<Vector3> m_Vertices = new List<Vector3>();
        private List<int> m_RoadTriangles = new List<int>();
        private List<int> m_JunctionTriangles = new List<int>();
        private List<Vector3> m_UVs = new List<Vector3>();   // (u·q, v·q, q): projective, see AddStripUVs
        private List<Color> m_Colors = new List<Color>();

        private readonly Dictionary<int, float2> m_SplineTrimRanges = new Dictionary<int, float2>();
        private readonly List<JunctionPad> m_JunctionPads = new List<JunctionPad>();

        private struct JunctionArm
        {
            public int splineIndex;
            public float t;            // spline end touching the junction (0 or 1)
            public float3 pos;         // centerline point at the junction
            public float3 outward;     // direction leaving the junction
            public float angle;
            public float width;        // this road's half-width
            public float capT;         // where the road strip stops and the pad takes over
            public float3 capOutward;
            public float3 capLeft;     // left/right relative to outward, not spline direction
            public float3 capRight;
        }

        private struct JunctionPad
        {
            public Vector3 center;
            public List<Vector3> boundary;
            public float uvScale;      // UV per unit, matching the connected roads' texel density
            public float width;        // average half-width of the connected roads
            public List<Vector3> bendLeft;   // two-road bends: edges of a marked strip from cap to cap
            public List<Vector3> bendRight;
        }

        public float JunctionSnapDistance => m_JunctionSnapDistance;
        public float RoadWidth => m_Width;

        public float ConnectDistance =>
            m_ConnectDistance > 0f ? m_ConnectDistance : math.max(m_JunctionSnapDistance, m_Width * 1.5f);

        /// <summary>
        /// A new connection closer than one road width to an existing junction joins it instead of
        /// forming its own; farther away it gets a separate junction so the new road is not bent.
        /// </summary>
        public float JunctionMergeDistance => math.min(m_JunctionSnapDistance, m_Width * 2f);

        /// <summary>
        /// Half-width of one road. Stored on the spline itself so it survives saving and editing;
        /// splines without their own value use the track default.
        /// </summary>
        public float GetRoadWidth(Spline spline)
        {
            if (spline != null && spline.TryGetFloatData(k_RoadWidthKey, out SplineData<float> data) && data.DefaultValue > 0f)
                return data.DefaultValue;
            return m_Width;
        }

        public float GetRoadWidth(int splineIndex) => GetRoadWidth(splineContainer.Splines[splineIndex]);

        /// <summary>Gives one road its own half-width; zero or less returns it to the track default.</summary>
        public void SetRoadWidth(Spline spline, float width)
        {
            if (spline == null) return;
            if (width > 0f)
                spline.GetOrCreateFloatData(k_RoadWidthKey).DefaultValue = width;
            else
                spline.RemoveFloatData(k_RoadWidthKey);
            m_NetworkDirty = true;
        }

        private void OnEnable()
        {
            Spline.Changed += OnSplineChanged;
            SplineContainer.SplineAdded += OnContainerChanged;
            SplineContainer.SplineRemoved += OnContainerChanged;
            // Junctions are derived from the splines; rebuild them after loading or a script reload.
            m_NetworkDirty = true;
        }

        private void OnDisable()
        {
            Spline.Changed -= OnSplineChanged;
            SplineContainer.SplineAdded -= OnContainerChanged;
            SplineContainer.SplineRemoved -= OnContainerChanged;
        }

        private void OnSplineChanged(Spline spline, int knotIndex, SplineModification modification)
        {
            if (IndexOfSpline(spline) >= 0)
                m_NetworkDirty = true;
        }

        private void OnContainerChanged(SplineContainer container, int index)
        {
            if (container == splineContainer)
                m_NetworkDirty = true;
        }

        // Moving, editing, connecting or disconnecting a road re-detects junctions once per frame.
        private void LateUpdate()
        {
            if (!m_NetworkDirty) return;
            m_NetworkDirty = false;
            AutoDetectJunctions();
        }

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
        /// Finds the nearest start/end knot among existing splines (optionally excluding one),
        /// within maxDistance or the junction snap distance when none is given.
        /// </summary>
        public bool TrySnapToNearestEndpoint(float3 localPosition, int excludeSplineIndex, out float3 snappedLocal,
            float maxDistance = -1f)
        {
            snappedLocal = localPosition;
            if (splineContainer == null || splineContainer.Splines == null)
                return false;

            float bestDist = maxDistance > 0f ? maxDistance : m_JunctionSnapDistance;
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
        /// True when the connecting spline's drawing tip is running into another road.
        /// </summary>
        public bool WouldCollideWithExistingRoad(int connectingIndex)
        {
            return TryFindRoadHit(connectingIndex, atStart: false, out _, out _, out _);
        }

        /// <summary>
        /// Snaps a drawn spline into the network: endpoint merge, or mid-road T-junction
        /// (insert knot, split hit road, snap tip, link knots). Each end connects on its own,
        /// so a stroke can leave one road and join another.
        /// </summary>
        public bool ConnectSplineToExistingRoads(int connectingIndex)
        {
            if (splineContainer == null || connectingIndex < 0 || connectingIndex >= splineContainer.Splines.Count)
                return false;

            var connecting = splineContainer.Splines[connectingIndex];
            if (connecting == null || connecting.Count < 2)
                return false;

            // Freeze AutoSmooth tangents so snapping / splitting cannot reshape existing roads.
            BakeAllSplineTangents();

            // Tip first: a mid-road split renumbers splines, so each end looks its index up again.
            bool connected = ConnectSplineEnd(connecting, atStart: false);
            connected |= ConnectSplineEnd(connecting, atStart: true);
            return connected;
        }

        private bool ConnectSplineEnd(Spline connecting, bool atStart)
        {
            int connectingIndex = IndexOfSpline(connecting);
            if (connectingIndex < 0 || connecting.Count < 2)
                return false;

            // Decide from where the stroke actually meets a road. Snapping to any endpoint near the
            // stroke end first would drag new roads onto a neighbouring junction and bend them.
            if (TryFindRoadHit(connectingIndex, atStart, out int hitSplineIndex, out float hitT, out float3 hitPos))
                return FormJunctionAtHit(connectingIndex, atStart, hitSplineIndex, hitT, hitPos);

            // Dead ends the stroke heads at are handled above; here only practically touching ends join.
            float3 endPos = connecting[atStart ? 0 : connecting.Count - 1].Position;
            if (TrySnapToNearestEndpoint(endPos, connectingIndex, out float3 snapped, JunctionMergeDistance))
            {
                SnapConnectingEnd(connecting, atStart, snapped);
                return true;
            }

            return false;
        }

        private void SnapConnectingEnd(Spline connecting, bool atStart, float3 position)
        {
            if (atStart)
            {
                TruncateCollapsedKnotsFromStart(connecting, position);
                SnapTipWithoutDeform(connecting, 0, position);
            }
            else
            {
                TruncateCollapsedKnotsFromEnd(connecting, position);
                SnapTipWithoutDeform(connecting, connecting.Count - 1, position);
            }
        }

        private bool FormJunctionAtHit(
            int connectingIndex,
            bool tipIsStart,
            int hitSplineIndex,
            float hitT,
            float3 hitPos)
        {
            var connecting = splineContainer.Splines[connectingIndex];
            var hitSpline = splineContainer.Splines[hitSplineIndex];

            // Near an existing endpoint (a dead end or another junction) — join it, snapping the new
            // tip only. Measured in distance: a fraction of t is huge on long roads and tiny on the
            // short pieces between junctions. Never rewrite the existing road's tangents.
            float3 firstPoint = hitSpline[0].Position;
            float3 lastPoint = hitSpline[hitSpline.Count - 1].Position;
            float dFirst = math.distance(hitPos, firstPoint);
            float dLast = math.distance(hitPos, lastPoint);
            if (math.min(dFirst, dLast) <= JunctionMergeDistance)
            {
                SnapConnectingEnd(connecting, tipIsStart, dFirst <= dLast ? firstPoint : lastPoint);
                return true;
            }

            // Mid-road hit → insert knot and split for topology, but keep the original curve.
            BakeSplineTangents(splineContainer.Splines[hitSplineIndex]);
            int knotIndex = InsertKnotAtNormalizedT(splineContainer.Splines[hitSplineIndex], hitT);
            if (knotIndex < 0)
                return false;

            var splitStart = SplineUtility.SplitSplineOnKnot(
                splineContainer,
                new SplineKnotIndex(hitSplineIndex, knotIndex));

            // After split: first half ends at junction; second half starts at junction.
            // Leave those knots alone so the through-road does not warp.
            var firstHalf = splineContainer.Splines[hitSplineIndex];
            // Splitting does not carry per-spline data over; keep both halves the same width.
            if (firstHalf.TryGetFloatData(k_RoadWidthKey, out SplineData<float> widthData))
                SetRoadWidth(splineContainer.Splines[splitStart.Spline], widthData.DefaultValue);
            connectingIndex = IndexOfSpline(connecting);
            if (connectingIndex < 0)
                return false;

            // Snap onto the inserted knot itself so every end at this junction coincides exactly.
            SnapConnectingEnd(connecting, tipIsStart, firstHalf[firstHalf.Count - 1].Position);

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
        /// The road one end of the connecting spline runs into, and where. Auto-connect stops the
        /// stroke up to ConnectDistance short, so the stroke is extended along its approach direction
        /// to find the real crossing; the road crossed soonest wins. A road merely running alongside
        /// within ConnectDistance must not capture the stroke, so without a crossing only a road the
        /// end already overlaps, or a dead end it runs into, counts.
        /// </summary>
        private bool TryFindRoadHit(
            int connectingIndex,
            bool atStart,
            out int hitSplineIndex,
            out float hitT,
            out float3 hitPos)
        {
            hitSplineIndex = -1;
            hitT = 0f;
            hitPos = float3.zero;

            var connecting = splineContainer.Splines[connectingIndex];
            if (connecting == null || connecting.Count < 2)
                return false;

            float3 origin = connecting[atStart ? 0 : connecting.Count - 1].Position;
            float reach = ConnectDistance;
            bool hasDirection = TryGetApproachDirection(connecting, atStart, out float3 direction);

            float bestAlong = float.MaxValue;
            int fallbackIndex = -1;
            float fallbackDist = float.MaxValue;
            float fallbackT = 0f;
            float3 fallbackPos = float3.zero;

            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                if (s == connectingIndex) continue;

                var other = splineContainer.Splines[s];
                if (other == null || other.Count < 2) continue;

                float dist = SplineUtility.GetNearestPoint(other, origin, out float3 nearest, out float tNear);
                if (dist > reach) continue;

                if (hasDirection &&
                    TryRaycastSplineXZ(other, origin, direction, reach, reach * 3f,
                        out float rayT, out float3 rayPos, out float along))
                {
                    if (math.abs(along) < bestAlong)
                    {
                        bestAlong = math.abs(along);
                        hitSplineIndex = s;
                        hitT = rayT;
                        hitPos = rayPos;
                    }
                    continue;
                }

                // Any point beside a road's end has that end as its nearest point, so a dead end only
                // counts when the stroke is heading at it, or already touching it.
                bool overlapping = dist <= m_Width * 2f;
                bool nearestIsEnd = math.distance(nearest, other[0].Position) < 1e-3f ||
                                    math.distance(nearest, other[other.Count - 1].Position) < 1e-3f;
                bool headingAtEnd = hasDirection && dist > 1e-4f &&
                                    math.dot(direction, math.normalizesafe(nearest - origin)) >= 0.5f;
                bool atDeadEnd = nearestIsEnd && (headingAtEnd || dist <= JunctionMergeDistance);
                if ((overlapping || atDeadEnd) && dist < fallbackDist)
                {
                    fallbackDist = dist;
                    fallbackIndex = s;
                    fallbackT = tNear;
                    fallbackPos = nearest;
                }
            }

            if (hitSplineIndex < 0 && fallbackIndex >= 0)
            {
                hitSplineIndex = fallbackIndex;
                hitT = fallbackT;
                hitPos = fallbackPos;
            }

            return hitSplineIndex >= 0;
        }

        /// <summary>
        /// Flat direction the stroke travels as it leaves through this end, averaged over a short
        /// stretch so knot jitter from the drawing lerp does not skew it.
        /// </summary>
        private bool TryGetApproachDirection(Spline connecting, bool atStart, out float3 direction)
        {
            direction = float3.zero;
            float length = connecting.GetLength();
            if (length < 1e-4f)
                return false;

            float back = math.clamp(ConnectDistance / length, 0.01f, 0.25f);
            float3 inner = connecting.EvaluatePosition(atStart ? back : 1f - back);
            float3 end = connecting[atStart ? 0 : connecting.Count - 1].Position;

            direction = end - inner;
            direction.y = 0f;
            direction = math.normalizesafe(direction);
            return math.lengthsq(direction) > 0.5f;
        }

        /// <summary>
        /// Intersects a ray with a spline's centerline on the ground plane. Accepts hits slightly
        /// behind the origin so an end that overshot the centerline is pulled back onto it.
        /// </summary>
        private static bool TryRaycastSplineXZ(
            Spline spline,
            float3 origin,
            float3 direction,
            float maxBehind,
            float maxAhead,
            out float hitT,
            out float3 hitPos,
            out float hitAlong)
        {
            hitT = 0f;
            hitPos = float3.zero;
            hitAlong = 0f;

            int samples = math.clamp((int)(spline.GetLength() / 0.05f), 16, 512);
            float bestAbs = float.MaxValue;
            float prevT = 0f;
            float3 prev = spline.EvaluatePosition(0f);

            for (int i = 1; i <= samples; i++)
            {
                float t = i / (float)samples;
                float3 next = spline.EvaluatePosition(t);
                float3 seg = next - prev;

                float denom = direction.x * seg.z - direction.z * seg.x;
                if (math.abs(denom) > 1e-6f)
                {
                    float3 diff = prev - origin;
                    float along = (diff.x * seg.z - diff.z * seg.x) / denom;
                    float u = (diff.x * direction.z - diff.z * direction.x) / denom;

                    if (u >= 0f && u <= 1f && along >= -maxBehind && along <= maxAhead &&
                        math.abs(along) < bestAbs)
                    {
                        bestAbs = math.abs(along);
                        hitAlong = along;
                        hitT = math.lerp(prevT, t, u);
                        hitPos = math.lerp(prev, next, u);
                    }
                }

                prev = next;
                prevT = t;
            }

            return bestAbs < float.MaxValue;
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

        private void BakeAllSplineTangents()
        {
            if (splineContainer == null) return;
            for (int s = 0; s < splineContainer.Splines.Count; s++)
                BakeSplineTangents(splineContainer.Splines[s]);
        }

        /// <summary>
        /// Converts AutoSmooth knots to Continuous using their current handles so later
        /// edits cannot ripple through the whole road.
        /// </summary>
        private static void BakeSplineTangents(Spline spline)
        {
            if (spline == null) return;
            for (int i = 0; i < spline.Count; i++)
            {
                if (spline.GetTangentMode(i) != TangentMode.AutoSmooth)
                    continue;

                var knot = spline[i];
                spline.SetTangentMode(i, TangentMode.Continuous);
                spline.SetKnot(i, knot);
            }
        }

        private int IndexOfSpline(Spline spline)
        {
            if (splineContainer == null || spline == null) return -1;
            for (int i = 0; i < splineContainer.Splines.Count; i++)
            {
                if (splineContainer.Splines[i] == spline)
                    return i;
            }
            return -1;
        }

        /// <summary>
        /// Moves only the joining tip of the new road. Linear mode keeps the last segment
        /// straight; existing roads are never touched.
        /// </summary>
        private static void SnapTipWithoutDeform(Spline spline, int index, float3 position)
        {
            if (spline == null || index < 0 || index >= spline.Count) return;

            spline.SetTangentMode(index, TangentMode.Linear);
            var knot = spline[index];
            knot.Position = position;
            spline.SetKnot(index, knot);

            // Moving the tip leaves a straight cut that can kink against the next kept knot
            // tighter than the road is wide, folding the strip. Re-smooth that one knot of the
            // new road so the bend spreads over both of its segments.
            if (spline.Count > 2)
                spline.SetTangentMode(index == 0 ? 1 : index - 1, TangentMode.AutoSmooth);
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
            if (splineContainer == null || splineContainer.Splines == null || splineSampler == null) return;

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

            BuildJunctionData();

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
                float width = GetRoadWidth(spline);

                // U runs across the road, V along it in road widths. V is measured on the real
                // centerline, so markings never stretch where t is uneven and scale with the width.
                float vScale = m_TextureScale / math.max(2f * width, 1e-4f);
                float distance = 0f;
                Vector3 previousCenter = Vector3.zero;
                Vector3 previousLeft = Vector3.zero;
                Vector3 previousRight = Vector3.zero;

                for (int i = 0; i <= resolution; i++)
                {
                    // The last section uses tEnd exactly so it matches the junction cap bit for bit.
                    float t = i == resolution ? tEnd : math.lerp(tStart, tEnd, i / (float)resolution);

                    splineSampler.TryGetSplineWidthEdges(s, t, width,
                        out float3 centerEdge, out _, out float3 leftEdge, out float3 rightEdge);

                    Vector3 left = worldToLocal.MultiplyPoint3x4(leftEdge);
                    Vector3 right = worldToLocal.MultiplyPoint3x4(rightEdge);

                    // Around a bend tighter than the road is wide, the inner edge runs backwards and
                    // would fold the strip over itself (flipped triangles, overlapping surface). Hold
                    // an edge in place whenever its new triangle would face the wrong way, so the inner
                    // edge pivots instead of folding. End sections are left alone so junction caps match.
                    if (i > 0 && i < resolution)
                    {
                        if (FacingUp(previousLeft, left, previousRight) <= 0f)
                            left = previousLeft;
                        if (FacingUp(previousRight, left, right) <= 0f)
                            right = previousRight;
                    }
                    previousLeft = left;
                    previousRight = right;

                    Vector3 center = worldToLocal.MultiplyPoint3x4(centerEdge);
                    if (i > 0)
                        distance += Vector3.Distance(center, previousCenter);
                    previousCenter = center;

                    m_Vertices.Add(left);
                    m_Vertices.Add(right);

                    AddStripUVs(m_UVs, distance * vScale, Vector3.Distance(left, right) / (2f * width));

                    // Alpha 1 lets the road shader paint lane markings on strips.
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

            foreach (var pad in m_JunctionPads)
            {
                BuildJunctionMesh(pad, m_Vertices, m_RoadTriangles, m_UVs, m_Colors);
            }

            if (m_Vertices.Count > 0)
            {
                m_Mesh.Clear();
                m_Mesh.indexFormat = m_Vertices.Count > 65535
                    ? UnityEngine.Rendering.IndexFormat.UInt32
                    : UnityEngine.Rendering.IndexFormat.UInt16;
                m_Mesh.SetVertices(m_Vertices);

                m_Mesh.subMeshCount = 1;
                m_Mesh.SetTriangles(m_RoadTriangles, 0);

                m_Mesh.SetUVs(0, m_UVs);
                m_Mesh.SetColors(m_Colors);

                // Roads and junctions are one flat surface built in a world-up frame, so every
                // vertex shares the same normal: no shading seams where strips meet pads.
                Vector3 up = transform.InverseTransformDirection(Vector3.up).normalized;
                var normals = new Vector3[m_Vertices.Count];
                for (int i = 0; i < normals.Length; i++)
                    normals[i] = up;
                m_Mesh.normals = normals;
                m_Mesh.RecalculateBounds();
            }
            else
            {
                m_Mesh.Clear();
            }
        }

        /// <summary>
        /// Strip UVs for one cross-section, stored projectively as (u·q, v·q, q) with q the section's
        /// width relative to the road (1 on a normal strip). Where a strip tapers (width changes or a
        /// held inner edge), plain UVs would zig-zag the markings along each quad's diagonal; dividing
        /// by q in the shader keeps lines of constant u straight.
        /// </summary>
        private static void AddStripUVs(List<Vector3> uvs, float v, float q)
        {
            q = math.max(q, 1e-3f);
            uvs.Add(new Vector3(0f, v * q, q));
            uvs.Add(new Vector3(q, v * q, q));
        }

        // Up component of a triangle's normal with this mesh's winding; positive means it faces up.
        private static float FacingUp(Vector3 a, Vector3 b, Vector3 c)
        {
            return Vector3.Cross(b - a, c - a).y;
        }

        private void BuildJunctionData()
        {
            m_SplineTrimRanges.Clear();
            m_JunctionPads.Clear();

            if (splineContainer == null) return;

            for (int s = 0; s < splineContainer.Splines.Count; s++)
                m_SplineTrimRanges[s] = new float2(0f, 1f);

            foreach (var group in m_JunctionGroups)
            {
                if (!TryBuildJunctionArms(group, out List<JunctionArm> arms))
                    continue;

                // A straight continuation of the same width already meets flush; a pad there would only
                // interrupt the lane markings. Any bend or width change gets a proper pad, otherwise the
                // two perpendicular ends leave a wedge gap on the outside of the bend.
                if (arms.Count == 2 && AreNearlyOpposite(arms[0].outward, arms[1].outward) &&
                    math.abs(arms[0].width - arms[1].width) < 1e-4f)
                    continue;

                bool capsValid = true;
                for (int i = 0; i < arms.Count && capsValid; i++)
                {
                    var arm = arms[i];
                    capsValid = ComputeArmCap(ref arm, GetArmClearance(arms, i));
                    arms[i] = arm;
                }
                if (!capsValid) continue;

                // Every arm stops at its cap. The cap is the strip's last cross-section, so the
                // pad boundary shares those exact vertices and no gap or overlap can form.
                foreach (var arm in arms)
                {
                    float2 range = m_SplineTrimRanges[arm.splineIndex];
                    if (arm.t <= 0.5f)
                        range.x = math.max(range.x, arm.capT);
                    else
                        range.y = math.min(range.y, arm.capT);
                    m_SplineTrimRanges[arm.splineIndex] = range;
                }

                m_JunctionPads.Add(BuildJunctionPad(arms));
            }
        }

        private bool TryBuildJunctionArms(List<JunctionBuilder.JunctionInfo> junctions, out List<JunctionArm> arms)
        {
            arms = new List<JunctionArm>();
            if (junctions == null || splineSampler == null) return false;

            foreach (var junction in junctions)
            {
                if (junction.splineIndex < 0 || junction.splineIndex >= splineContainer.Splines.Count)
                    continue;

                float width = GetRoadWidth(junction.splineIndex);
                if (!splineSampler.TryGetSplineWidthEdges(
                        junction.splineIndex, junction.t, width,
                        out float3 pos, out float3 tan, out _, out _))
                    continue;

                float3 outward = junction.t <= 0.5f ? tan : -tan;
                arms.Add(new JunctionArm
                {
                    splineIndex = junction.splineIndex,
                    t = junction.t,
                    pos = pos,
                    outward = outward,
                    angle = math.atan2(outward.z, outward.x),
                    width = width
                });
            }

            if (arms.Count < 2) return false;

            arms.Sort((a, b) => a.angle.CompareTo(b.angle));
            return true;
        }

        /// <summary>
        /// Distance from the junction where this arm's edges clear both neighbours,
        /// plus room for the curved blend.
        /// </summary>
        private float GetArmClearance(List<JunctionArm> arms, int index)
        {
            int count = arms.Count;
            var arm = arms[index];
            var prev = arms[(index - 1 + count) % count];
            var next = arms[(index + 1) % count];
            float prevGap = WrapAngle(arm.angle - prev.angle);
            float nextGap = WrapAngle(next.angle - arm.angle);
            return math.max(
                       ClearanceForGap(arm.width, prev.width, prevGap),
                       ClearanceForGap(arm.width, next.width, nextGap))
                   + arm.width * m_JunctionInsetScale;
        }

        /// <summary>
        /// Distance along a road of half-width w at which its edge crosses the facing edge of a
        /// neighbour of half-width wOther leaving the junction `gap` radians away:
        /// (wOther + w cos gap) / sin gap, which is w / tan(gap / 2) for equal widths.
        /// </summary>
        private static float ClearanceForGap(float w, float wOther, float gap)
        {
            if (gap >= math.PI) return 0f;
            float clearance = (wOther + w * math.cos(gap)) / math.max(math.sin(gap), math.sin(0.05f));
            // Near-opposite roads of different widths have almost parallel edges that only meet far
            // away; they cannot overlap there, so a short taper is enough.
            if (gap > math.PI * 0.5f)
                clearance = math.min(clearance, math.max(w, wOther));
            return math.max(0f, clearance);
        }

        private static float WrapAngle(float angle)
        {
            angle %= 2f * math.PI;
            return angle < 0f ? angle + 2f * math.PI : angle;
        }

        private bool ComputeArmCap(ref JunctionArm arm, float clearance)
        {
            var spline = splineContainer.Splines[arm.splineIndex];
            float length = math.max(spline.GetLength(), 1e-4f);
            float inset = math.clamp(clearance / length, 0.01f, 0.45f);
            bool alongSpline = arm.t <= 0.5f;
            arm.capT = alongSpline ? inset : 1f - inset;

            if (!splineSampler.TryGetSplineWidthEdges(
                    arm.splineIndex, arm.capT, arm.width,
                    out _, out float3 tan, out float3 left, out float3 right))
                return false;

            // Sampler edges follow spline direction; flip them for arms that leave against it
            // so neighbouring arms always pair facing edges.
            arm.capOutward = alongSpline ? tan : -tan;
            arm.capLeft = alongSpline ? left : right;
            arm.capRight = alongSpline ? right : left;
            return true;
        }

        /// <summary>
        /// Pad outline: each arm's cap, joined to the next arm by a quadratic Lerp curve whose
        /// control point is where the two road edges meet, so the curb leaves each road tangent to it.
        /// </summary>
        private JunctionPad BuildJunctionPad(List<JunctionArm> arms)
        {
            Matrix4x4 worldToLocal = transform.worldToLocalMatrix;

            float3 center = float3.zero;
            foreach (var arm in arms)
                center += arm.pos;
            center /= arms.Count;

            float curveLimit = 0f;
            float maxWidth = 0f;
            float widthSum = 0f;
            foreach (var arm in arms)
            {
                curveLimit = math.max(curveLimit, math.distance(arm.capLeft, center));
                maxWidth = math.max(maxWidth, arm.width);
                widthSum += arm.width;
            }
            curveLimit += maxWidth * 2f;

            int segments = math.max(1, m_JunctionCurveSegments);
            var boundary = new List<Vector3>(arms.Count * (segments + 1));

            // Curb between each arm's left edge and the next arm's right edge.
            var controls = new Vector3[arms.Count];
            for (int i = 0; i < arms.Count; i++)
                controls[i] = CurbControl(arms[i], arms[(i + 1) % arms.Count], center, curveLimit);

            for (int i = 0; i < arms.Count; i++)
            {
                var arm = arms[i];
                var next = arms[(i + 1) % arms.Count];

                boundary.Add(worldToLocal.MultiplyPoint3x4(arm.capRight));
                boundary.Add(worldToLocal.MultiplyPoint3x4(arm.capLeft));

                for (int k = 1; k < segments; k++)
                {
                    float s = k / (float)segments;
                    boundary.Add(worldToLocal.MultiplyPoint3x4(QuadraticLerp(arm.capLeft, controls[i], next.capRight, s)));
                }
            }

            var pad = new JunctionPad
            {
                center = worldToLocal.MultiplyPoint3x4(center),
                boundary = boundary,
                uvScale = m_TextureScale / math.max(2f * widthSum / arms.Count, 1e-4f),
                width = widthSum / arms.Count
            };

            // A bend or width change between just two roads is not an intersection: loft it as a road
            // strip from cap to cap, each cross-section lerped between the two curbs, so the lane
            // markings follow the curve instead of stopping at a plain pad.
            if (arms.Count == 2)
            {
                int sections = segments * 2;
                pad.bendLeft = new List<Vector3>(sections + 1);
                pad.bendRight = new List<Vector3>(sections + 1);
                for (int k = 0; k <= sections; k++)
                {
                    float s = k / (float)sections;
                    pad.bendLeft.Add(worldToLocal.MultiplyPoint3x4(
                        QuadraticLerp(arms[0].capRight, controls[1], arms[1].capLeft, s)));
                    pad.bendRight.Add(worldToLocal.MultiplyPoint3x4(
                        QuadraticLerp(arms[0].capLeft, controls[0], arms[1].capRight, s)));
                }
            }

            return pad;
        }

        /// <summary>
        /// Control point of the curb from one arm's left edge to the next arm's right edge: where the
        /// two edge lines meet, so the curb leaves each road tangent to it.
        /// </summary>
        private static Vector3 CurbControl(JunctionArm arm, JunctionArm next, float3 center, float curveLimit)
        {
            // The edges must meet behind both caps (on the junction side). On the far side of a
            // slightly curved through road the near-parallel edges can meet just past a cap, which
            // would bend the curb back over that road strip; keep those edges straight.
            if (TryIntersectLinesXZ(arm.capLeft, arm.capOutward, next.capRight, next.capOutward, out float3 hit)
                && math.distance(hit, center) <= curveLimit
                && math.dot(hit - arm.capLeft, arm.capOutward) <= 0f
                && math.dot(hit - next.capRight, next.capOutward) <= 0f)
            {
                return hit;
            }
            return Vector3.Lerp(arm.capLeft, next.capRight, 0.5f);
        }

        private static Vector3 QuadraticLerp(Vector3 p0, Vector3 control, Vector3 p2, float s)
        {
            return Vector3.Lerp(Vector3.Lerp(p0, control, s), Vector3.Lerp(control, p2, s), s);
        }

        private static bool TryIntersectLinesXZ(float3 p1, float3 d1, float3 p2, float3 d2, out float3 result)
        {
            float denom = d1.x * d2.z - d1.z * d2.x;
            if (math.abs(denom) < 1e-6f)
            {
                result = (p1 + p2) * 0.5f;
                return false;
            }

            float t = ((p2.x - p1.x) * d2.z - (p2.z - p1.z) * d2.x) / denom;
            result = p1 + d1 * t;
            result.y = (p1.y + p2.y) * 0.5f;
            return true;
        }

        private static bool AreNearlyOpposite(float3 a, float3 b)
        {
            return math.dot(a, b) < -0.996f;
        }

        private void BuildJunctionMesh(
            JunctionPad pad,
            List<Vector3> vertices,
            List<int> triangles,
            List<Vector3> uvs,
            List<Color> colors)
        {
            if (pad.bendLeft != null)
            {
                BuildBendStrip(pad, vertices, triangles, uvs, colors);
                return;
            }

            int count = pad.boundary.Count;
            if (count < 3) return;

            // Pads get their own planar UVs at the roads' texel density instead of stretched strip
            // UVs, and alpha 0 tells the road shader to leave lane markings off the junction.
            var padColor = new Color(1f, 1f, 1f, 0f);

            int centerIndex = vertices.Count;
            vertices.Add(pad.center);
            uvs.Add(new Vector3(pad.center.x * pad.uvScale, pad.center.z * pad.uvScale, 1f));
            colors.Add(padColor);

            int first = vertices.Count;
            foreach (var c in pad.boundary)
            {
                vertices.Add(c);
                uvs.Add(new Vector3(c.x * pad.uvScale, c.z * pad.uvScale, 1f));
                colors.Add(padColor);
            }

            // Boundary runs counter-clockwise by angle, same winding as before.
            for (int i = 0; i < count; i++)
            {
                triangles.Add(centerIndex);
                triangles.Add(first + (i + 1) % count);
                triangles.Add(first + i);
            }
        }

        // Same layout as a road strip: U across, V along in road widths, markings on (alpha 1).
        private static void BuildBendStrip(
            JunctionPad pad,
            List<Vector3> vertices,
            List<int> triangles,
            List<Vector3> uvs,
            List<Color> colors)
        {
            int offset = vertices.Count;
            int last = pad.bendLeft.Count - 1;
            float distance = 0f;
            Vector3 previousCenter = Vector3.zero;
            Vector3 previousLeft = Vector3.zero;
            Vector3 previousRight = Vector3.zero;

            for (int k = 0; k <= last; k++)
            {
                Vector3 left = pad.bendLeft[k];
                Vector3 right = pad.bendRight[k];
                Vector3 center = (left + right) * 0.5f;
                if (k > 0)
                    distance += Vector3.Distance(center, previousCenter);
                previousCenter = center;

                if (k > 0 && k < last)
                {
                    if (FacingUp(previousLeft, left, previousRight) <= 0f)
                        left = previousLeft;
                    if (FacingUp(previousRight, left, right) <= 0f)
                        right = previousRight;
                }
                previousLeft = left;
                previousRight = right;

                vertices.Add(left);
                vertices.Add(right);
                AddStripUVs(uvs, distance * pad.uvScale, Vector3.Distance(left, right) / (2f * pad.width));
                colors.Add(Color.white);
                colors.Add(Color.white);

                if (k < last)
                {
                    int start = offset + k * 2;
                    triangles.Add(start);
                    triangles.Add(start + 2);
                    triangles.Add(start + 1);

                    triangles.Add(start + 1);
                    triangles.Add(start + 2);
                    triangles.Add(start + 3);
                }
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

            // Connected ends are snapped onto the same point, so only touching ends form a junction.
            // The larger snap radius chained neighbouring junctions into one distorted pad.
            float joinDistance = math.min(m_JunctionSnapDistance, m_Width * 0.5f);
            float snapSq = joinDistance * joinDistance;
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
