using UnityEngine;
using System.Collections;
using System.Collections.Generic;

namespace Unity.Splines
{
    public class SplineFollower : MonoBehaviour
    {
        [SerializeField] private float m_MoveSpeed = 5f;
        
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

        private IEnumerator MoveAlongPoints(List<Vector3> points)
        {
            // Set initial position to first point
            transform.position = points[0];

            int currentIndex = 0;
            int direction = 1;
            float elapsedInSegment = 0;

            while (true)
            {
                int nextIndex = currentIndex + direction;

                if (nextIndex >= points.Count || nextIndex < 0)
                {
                    direction *= -1;
                    nextIndex = currentIndex + (direction * 2); // Skip the point we just reached
                    
                    // Safety check for very short paths
                    if (nextIndex < 0 || nextIndex >= points.Count)
                    {
                        nextIndex = currentIndex + direction;
                    }
                }

                Vector3 startPos = points[currentIndex];
                Vector3 targetPos = points[nextIndex];
                float segmentDistance = Vector3.Distance(startPos, targetPos);
                
                // If points are coincident, skip segment
                if (segmentDistance < 0.001f)
                {
                    currentIndex = nextIndex;
                    continue;
                }

                float segmentDuration = segmentDistance / m_MoveSpeed;

                while (elapsedInSegment < segmentDuration)
                {
                    transform.position = Vector3.Lerp(startPos, targetPos, elapsedInSegment / segmentDuration);
                    elapsedInSegment += Time.deltaTime;
                    yield return null;
                }

                // Carry over the overshoot to the next segment
                elapsedInSegment -= segmentDuration;
                currentIndex = nextIndex;
            }
        }
    }
}
