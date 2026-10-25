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
        [Tooltip("Half-width given to newly drawn roads. Zero or less uses the SplineTrack default.")]
        [SerializeField] private float m_RoadWidth = 0f;

        private Spline m_CurrentSpline;
        private int m_CurrentSplineIndex = -1;
        private float3 m_LastLerpedPosition;
        private bool m_IsDragging = false;

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

                m_LastLerpedPosition = math.lerp(m_LastLerpedPosition, targetLocalPos, Time.deltaTime * m_LerpSpeed);

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

            if (m_SplineTrack != null && m_RoadWidth > 0f)
                m_SplineTrack.SetRoadWidth(m_CurrentSpline, m_RoadWidth);

            m_CurrentSpline.Add(new BezierKnot(position), TangentMode.AutoSmooth);
        }

        private bool AddKnotToSpline(float3 position)
        {
            if (m_CurrentSpline == null) return false;

            var lastKnot = m_CurrentSpline.Knots.ElementAt(m_CurrentSpline.Count - 1);
            if (math.distance(lastKnot.Position, position) < m_KnotSpacing)
            {
                return false;
            }

            m_CurrentSpline.Add(new BezierKnot(position), TangentMode.AutoSmooth);
            return true;
        }

        private void FinishCurrentSpline()
        {
            if (!m_IsDragging && m_CurrentSpline == null)
                return;

            if (m_CurrentSpline != null && m_CurrentSpline.Count >= 2 && m_SplineTrack != null)
            {
                m_SplineTrack.ConnectSplineToExistingRoads(m_CurrentSplineIndex);
                m_SplineTrack.AutoDetectJunctions();
            }
            else if (m_CurrentSpline != null && m_SplineTrack != null)
            {
                m_SplineTrack.RebuildMesh();
            }

            m_CurrentSpline = null;
            m_CurrentSplineIndex = -1;
            m_IsDragging = false;
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
        }
    }
}
