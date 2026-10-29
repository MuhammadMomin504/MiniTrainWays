using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using System.Collections;
using System.Collections.Generic;

namespace Unity.Splines
{
    public class SplineFollower : MonoBehaviour
    {
        // Junction ends are snapped onto each other, so matching ends coincide almost exactly.
        private const float k_JunctionTolerance = 0.05f;

        [SerializeField] private float m_MoveSpeed = 5f;

        private float m_ElapsedInSegment;

        public void InitAndStartMove(List<Vector3> worldPoints)
        {
            if (worldPoints == null || worldPoints.Count < 2)
            {
                if (worldPoints != null && worldPoints.Count == 1)
                    transform.position = worldPoints[0];
                return;
            }
            StartCoroutine(MoveAlongPoints(worldPoints));
        }

        /// <summary>
        /// Drives along the road network: at the end of a road it carries on onto another road
        /// that meets there, and turns back only at dead ends.
        /// </summary>
        public void InitAndStartMove(SplineContainer container, Spline spline, float pointSpacing)
        {
            if (container == null || spline == null || spline.Count < 2)
                return;
            StartCoroutine(MoveAlongNetwork(container, spline, Mathf.Max(0.01f, pointSpacing)));
        }

        private IEnumerator MoveAlongPoints(List<Vector3> points)
        {
            // Set initial position to first point
            transform.position = points[0];

            int currentIndex = 0;
            int direction = 1;
            m_ElapsedInSegment = 0;

            while (true)
            {
                int nextIndex = currentIndex + direction;

                // Turn back towards the neighbour of the end point we just reached
                if (nextIndex >= points.Count || nextIndex < 0)
                {
                    direction *= -1;
                    nextIndex = currentIndex + direction;
                }

                yield return MoveSegment(points[currentIndex], points[nextIndex]);
                currentIndex = nextIndex;
            }
        }

        private IEnumerator MoveAlongNetwork(SplineContainer container, Spline spline, float pointSpacing)
        {
            bool forward = true;
            m_ElapsedInSegment = 0;
            transform.position = container.transform.TransformPoint(spline[0].Position);

            while (true)
            {
                // Sample on entry so a road reshaped by newer strokes is followed as it is now.
                var points = SamplePoints(container, spline, forward, pointSpacing);
                for (int i = 0; i < points.Count - 1; i++)
                    yield return MoveSegment(points[i], points[i + 1]);

                // The road was removed (e.g. Clear All Splines); nothing left to drive on.
                if (IndexOfSpline(container, spline) < 0)
                {
                    Destroy(gameObject);
                    yield break;
                }

                bool arrivedAtEnd = forward;
                if (TryPickNextRoad(container, spline, arrivedAtEnd, out Spline next, out bool enterAtStart))
                {
                    spline = next;
                    forward = enterAtStart;
                }
                else
                {
                    forward = !forward;
                }
            }
        }

        private IEnumerator MoveSegment(Vector3 startPos, Vector3 targetPos)
        {
            float segmentDistance = Vector3.Distance(startPos, targetPos);

            // If points are coincident, skip segment
            if (segmentDistance < 0.001f)
                yield break;

            float segmentDuration = segmentDistance / m_MoveSpeed;

            while (m_ElapsedInSegment < segmentDuration)
            {
                transform.position = Vector3.Lerp(startPos, targetPos, m_ElapsedInSegment / segmentDuration);
                m_ElapsedInSegment += Time.deltaTime;
                yield return null;
            }

            // Carry over the overshoot to the next segment
            m_ElapsedInSegment -= segmentDuration;
        }

        private static List<Vector3> SamplePoints(SplineContainer container, Spline spline, bool forward, float pointSpacing)
        {
            // Sample the curve, not just the knots, so the follower stays on the road through bends.
            int segments = Mathf.Max(1, Mathf.CeilToInt(spline.GetLength() / pointSpacing));
            var points = new List<Vector3>(segments + 1);
            for (int i = 0; i <= segments; i++)
            {
                float t = (float)i / segments;
                float3 localPos = spline.EvaluatePosition(forward ? t : 1f - t);
                points.Add(container.transform.TransformPoint(localPos));
            }
            return points;
        }

        /// <summary>
        /// Picks a random road with an end at the junction just reached, other than the end we
        /// arrived on. Returns false at a dead end.
        /// </summary>
        private static bool TryPickNextRoad(SplineContainer container, Spline current, bool arrivedAtEnd,
            out Spline next, out bool enterAtStart)
        {
            float3 junction = arrivedAtEnd ? current[current.Count - 1].Position : current[0].Position;
            var candidates = new List<(Spline spline, bool atStart)>();

            foreach (var spline in container.Splines)
            {
                if (spline == null || spline.Count < 2)
                    continue;

                if ((spline != current || arrivedAtEnd) &&
                    math.distance(spline[0].Position, junction) <= k_JunctionTolerance)
                    candidates.Add((spline, true));

                if ((spline != current || !arrivedAtEnd) &&
                    math.distance(spline[spline.Count - 1].Position, junction) <= k_JunctionTolerance)
                    candidates.Add((spline, false));
            }

            if (candidates.Count == 0)
            {
                next = null;
                enterAtStart = false;
                return false;
            }

            var pick = candidates[UnityEngine.Random.Range(0, candidates.Count)];
            next = pick.spline;
            enterAtStart = pick.atStart;
            return true;
        }

        private static int IndexOfSpline(SplineContainer container, Spline spline)
        {
            for (int i = 0; i < container.Splines.Count; i++)
            {
                if (container.Splines[i] == spline)
                    return i;
            }
            return -1;
        }
    }
}
