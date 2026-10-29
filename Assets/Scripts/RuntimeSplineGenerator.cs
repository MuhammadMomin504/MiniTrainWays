using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using System.Linq;

namespace Unity.Splines
{
    public class RuntimeSplineGenerator : MonoBehaviour
    {
        [SerializeField] private SplineContainer m_SplineContainer;
        [SerializeField] private SplineTrack m_SplineTrack;
        [SerializeField] private float m_KnotSpacing = 1.0f;
        [SerializeField] private float m_LerpSpeed = 10.0f;
        [SerializeField] private LayerMask m_GroundLayer = -1;
        [SerializeField] private bool m_AutoConnectOnCollision = true;
        [SerializeField] private float m_MinLengthBeforeAutoConnect = 1.0f;
        [Tooltip("Largest direction change allowed between consecutive knots, in degrees. Sharper turns are eased to this limit.")]
        [Range(1f, 180f)]
        [SerializeField] private float m_MaxTurnAngle = 35f;
        [Tooltip("Smallest turning radius of a drawn road, in local units. Keep it just above the road half-width; larger values stop the road from following tight strokes. Zero or less disables the radius limit.")]
        [SerializeField] private float m_MinTurnRadius = 0.3f;
        [Tooltip("Half-width given to newly drawn roads. Zero or less uses the SplineTrack default.")]
        [SerializeField] private float m_RoadWidth = 0f;
        [Tooltip("Spawned on each finished road and moved along it. Leave empty to spawn nothing.")]
        [SerializeField] private SplineFollower m_FollowerPrefab;
        [SerializeField] private float m_FollowerPointSpacing = 0.25f;

        private Spline m_CurrentSpline;
        private int m_CurrentSplineIndex = -1;
        private float3 m_LastLerpedPosition;
        private bool m_IsDragging = false;
        // The current spline's last knot follows the cursor and is not yet committed.
        private bool m_HasTip;
        // Stroke distance not yet spent curving toward a cursor inside the turning circle.
        private float m_ArcBudget;
        private float3 m_LastStrokePosition;

        private const int k_MaxKnotsPerUpdate = 64;
        private const float k_MinTipLength = 0.01f;
        private const float k_TurnCatchUp = 2f;

        private void Start()
        {
            if (m_SplineContainer == null)
            {
                m_SplineContainer = GetComponent<SplineContainer>();
            }

            if (m_SplineTrack == null)
            {
                m_SplineTrack = GetComponent<SplineTrack>();
            }
        }

        private void Update()
        {
            if (Input.GetMouseButtonDown(0))
            {
                HandleMouseDown();
            }
            else if (Input.GetMouseButton(0))
            {
                HandleMouseDrag();
            }

            if (Input.GetMouseButtonUp(0))
            {
                FinishCurrentSpline();
            }

            if (Input.GetMouseButtonDown(1))
            {
                FinishCurrentSpline();
            }
        }

        private void HandleMouseDown()
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, 100f, m_GroundLayer))
            {
                Vector3 worldPos = hit.point;
                float3 localPos = m_SplineContainer.transform.InverseTransformPoint(worldPos);
                localPos.y = -2f;

                if (m_SplineTrack != null &&
                    m_SplineTrack.TrySnapToNearestEndpoint(localPos, -1, out float3 snappedStart,
                        m_SplineTrack.JunctionMergeDistance))
                {
                    localPos = snappedStart;
                    localPos.y = -2f;
                }

                m_LastLerpedPosition = localPos;
                m_IsDragging = true;
                StartNewSpline(localPos);

                if (m_SplineTrack != null)
                {
                    m_SplineTrack.RebuildMesh();
                }
            }
        }

        private void HandleMouseDrag()
        {
            if (m_CurrentSpline == null || !m_IsDragging) return;

            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out RaycastHit hit, 100f, m_GroundLayer))
            {
                Vector3 worldPos = hit.point;
                float3 targetLocalPos = m_SplineContainer.transform.InverseTransformPoint(worldPos);
                targetLocalPos.y = -2f;

                // Exponential smoothing keeps the same feel at any frame rate.
                m_LastLerpedPosition = math.lerp(m_LastLerpedPosition, targetLocalPos, 1f - math.exp(-m_LerpSpeed * Time.deltaTime));

                if (AddKnotToSpline(m_LastLerpedPosition))
                {
                    if (m_SplineTrack != null)
                    {
                        m_SplineTrack.RebuildMesh();
                    }

                    // Auto-finish when the stroke tip collides with an existing road
                    if (m_AutoConnectOnCollision &&
                        m_SplineTrack != null &&
                        m_CurrentSpline.GetLength() >= m_MinLengthBeforeAutoConnect &&
                        m_SplineTrack.WouldCollideWithExistingRoad(m_CurrentSplineIndex))
                    {
                        FinishCurrentSpline();
                    }
                }
            }
        }

        private void StartNewSpline(float3 position)
        {
            m_CurrentSpline = m_SplineContainer.AddSpline();
            m_CurrentSplineIndex = m_SplineContainer.Splines.Count - 1;
            m_HasTip = false;
            m_ArcBudget = 0f;
            m_LastStrokePosition = position;

            if (m_SplineTrack != null && m_RoadWidth > 0f)
                m_SplineTrack.SetRoadWidth(m_CurrentSpline, m_RoadWidth);

            m_CurrentSpline.Add(new BezierKnot(position), TangentMode.AutoSmooth);
        }

        // Commits a knot every m_KnotSpacing, then moves a live tip knot to the cursor so the road grows
        // continuously instead of jumping a whole spacing at a time. Returns true when the spline changed.
        private bool AddKnotToSpline(float3 position)
        {
            if (m_CurrentSpline == null) return false;

            // The arc around a sharp corner is longer than the stroke's own path, so it gets extra length.
            m_ArcBudget += math.distance(m_LastStrokePosition, position) * k_TurnCatchUp;
            m_LastStrokePosition = position;

            bool added = false;
            bool turning = false;
            for (int i = 0; i < k_MaxKnotsPerUpdate; i++)
            {
                int committed = CommittedKnotCount();
                float3 last = m_CurrentSpline[committed - 1].Position;
                if (math.distance(last, position) < m_KnotSpacing)
                    break;

                if (committed < 2 ||
                    !TryLimitTurn(m_CurrentSpline[committed - 2].Position, last, position, out float3 limited,
                        out bool outOfReach))
                {
                    CommitKnot(position);
                    added = true;
                    break;
                }

                // Turn too sharp: step one knot along the tightest allowed arc until the road faces the cursor.
                // A cursor inside the turning circle can't be reached yet, so the road keeps curving along
                // the arc only as far as the stroke has moved, instead of waiting and then jumping.
                if (outOfReach)
                {
                    turning = true;
                    if (m_ArcBudget < m_KnotSpacing)
                        break;
                    m_ArcBudget -= m_KnotSpacing;
                }

                CommitKnot(limited);
                added = true;
            }

            if (!turning)
                m_ArcBudget = 0f;

            return UpdateTip(position, turning) || added;
        }

        private int CommittedKnotCount()
        {
            return m_HasTip ? m_CurrentSpline.Count - 1 : m_CurrentSpline.Count;
        }

        // A tip knot becomes the committed knot in place, so neighbouring tangents don't pop.
        private void CommitKnot(float3 position)
        {
            if (m_HasTip)
            {
                m_CurrentSpline[m_CurrentSpline.Count - 1] = new BezierKnot(position);
                m_HasTip = false;
            }
            else
            {
                m_CurrentSpline.Add(new BezierKnot(position), TangentMode.AutoSmooth);
            }
        }

        // Places the tip toward the cursor. On a turn sharper than allowed, the tip rides the tightest arc
        // (the same one TryLimitTurn steps along), so committing it later doesn't change the road's shape.
        // While turning toward an unreachable cursor, the tip only reaches as far as the arc budget.
        private bool UpdateTip(float3 position, bool turning)
        {
            int committed = CommittedKnotCount();
            float3 last = m_CurrentSpline[committed - 1].Position;
            float length = math.distance(last.xz, position.xz);
            float maxChord = turning ? math.min(m_ArcBudget, m_KnotSpacing) : m_KnotSpacing;
            if (turning)
                length = math.min(length, maxChord);

            if (length < k_MinTipLength)
            {
                if (!m_HasTip) return false;
                m_CurrentSpline.RemoveAt(m_CurrentSpline.Count - 1);
                m_HasTip = false;
                return true;
            }

            float3 tip = position;
            if (committed >= 2)
            {
                float2 prevDir = math.normalizesafe(last.xz - m_CurrentSpline[committed - 2].Position.xz);
                float2 dir = math.normalizesafe(position.xz - last.xz);
                float angle = math.atan2(prevDir.x * dir.y - prevDir.y * dir.x, math.dot(prevDir, dir));
                float maxTurn = MaxTurn(m_KnotSpacing);
                float chord = math.min(length, maxChord);
                float radius = m_KnotSpacing / (2f * math.sin(maxTurn * 0.5f));
                // A chord of this length along the arc turns half the knot turn plus half its own arc angle.
                float allowed = maxTurn * 0.5f + math.asin(math.min(1f, chord / (2f * radius)));
                if (turning || math.abs(angle) > allowed)
                {
                    float2 xz = last.xz + Rotate(prevDir, math.sign(angle) * allowed) * chord;
                    tip = new float3(xz.x, position.y, xz.y);
                }
            }

            if (m_HasTip)
            {
                m_CurrentSpline[m_CurrentSpline.Count - 1] = new BezierKnot(tip);
            }
            else
            {
                m_CurrentSpline.Add(new BezierKnot(tip), TangentMode.AutoSmooth);
                m_HasTip = true;
            }
            return true;
        }

        // When the segment to candidate bends more than allowed from the previous segment, returns in limited
        // a point one knot spacing ahead, turned by the largest allowed angle, so the road eases into the turn.
        // outOfReach is set when candidate lies inside the turning circle and can't be reached yet.
        private bool TryLimitTurn(float3 prev, float3 last, float3 candidate, out float3 limited, out bool outOfReach)
        {
            limited = candidate;
            outOfReach = false;
            float2 prevDir = last.xz - prev.xz;
            float2 newDir = candidate.xz - last.xz;
            float prevLen = math.length(prevDir);
            float newLen = math.length(newDir);
            if (prevLen < 1e-4f || newLen < 1e-4f) return false;

            prevDir /= prevLen;
            newDir /= newLen;

            float cross = prevDir.x * newDir.y - prevDir.y * newDir.x;
            float angle = math.atan2(cross, math.dot(prevDir, newDir));
            float maxTurn = MaxTurn(m_KnotSpacing);
            if (math.abs(angle) <= maxTurn) return false;

            float side = angle >= 0f ? 1f : -1f;
            float radius = m_KnotSpacing / (2f * math.sin(maxTurn * 0.5f));
            // Chords along the arc turn by maxTurn each, so the arc's tangent at last sits half a turn past prevDir.
            float2 tangent = Rotate(prevDir, side * maxTurn * 0.5f);
            float2 center = last.xz + side * new float2(-tangent.y, tangent.x) * radius;
            outOfReach = math.distance(center, candidate.xz) < radius;

            float2 xz = last.xz + Rotate(prevDir, side * maxTurn) * m_KnotSpacing;
            limited = new float3(xz.x, candidate.y, xz.y);
            return true;
        }

        private static float2 Rotate(float2 v, float radians)
        {
            float cos = math.cos(radians);
            float sin = math.sin(radians);
            return new float2(v.x * cos - v.y * sin, v.x * sin + v.y * cos);
        }

        // Largest direction change, in radians, allowed over a segment of the given length.
        private float MaxTurn(float segmentLength)
        {
            float maxAngle = math.radians(m_MaxTurnAngle);
            if (m_MinTurnRadius > 0f)
                maxAngle = math.min(maxAngle, segmentLength / m_MinTurnRadius);
            return maxAngle;
        }

        private void FinishCurrentSpline()
        {
            if (!m_IsDragging && m_CurrentSpline == null)
                return;

            if (m_CurrentSpline != null && m_CurrentSpline.Count >= 2 && m_SplineTrack != null)
            {
                m_SplineTrack.ConnectSplineToExistingRoads(m_CurrentSplineIndex);
                m_SplineTrack.AutoDetectJunctions();
                SpawnFollower(m_CurrentSpline);
            }
            else if (m_CurrentSpline != null && m_SplineTrack != null)
            {
                m_SplineTrack.RebuildMesh();
            }

            m_CurrentSpline = null;
            m_CurrentSplineIndex = -1;
            m_IsDragging = false;
            m_HasTip = false;
        }

        private void SpawnFollower(Spline spline)
        {
            if (m_FollowerPrefab == null)
                return;

            Vector3 start = m_SplineContainer.transform.TransformPoint(spline[0].Position);
            var follower = Instantiate(m_FollowerPrefab, start, Quaternion.identity);
            follower.InitAndStartMove(m_SplineContainer, spline, m_FollowerPointSpacing);
        }

        [ContextMenu("Clear All Splines")]
        public void ClearAllSplines()
        {
            if (m_SplineContainer != null)
            {
                while (m_SplineContainer.Splines.Count > 0)
                {
                    m_SplineContainer.RemoveSplineAt(0);
                }
            }

            if (m_SplineTrack != null)
            {
                m_SplineTrack.ClearJunctions();
                m_SplineTrack.RebuildMesh();
            }

            m_CurrentSpline = null;
            m_CurrentSplineIndex = -1;
            m_IsDragging = false;
            m_HasTip = false;
        }
    }
}
