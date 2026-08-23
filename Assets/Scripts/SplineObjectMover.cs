using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using System.Collections;
using System.Collections.Generic;

namespace Unity.Splines
{
    public class SplineObjectMover : MonoBehaviour
    {
        [SerializeField] private SplineContainer m_SplineContainer;
        [SerializeField] private GameObject m_PrefabToMove;
        [SerializeField] private float m_MoveDuration = 0.5f;

        private void Start()
        {
            if (m_SplineContainer == null)
            {
                m_SplineContainer = GetComponent<SplineContainer>();
            }
        }

        private void Update()
        {
            if (Input.GetMouseButtonUp(0))
            {
                StartMoveAlongSpline();
            }
        }

        private void StartMoveAlongSpline()
        {
            if (m_SplineContainer == null || m_SplineContainer.Splines.Count == 0 || m_PrefabToMove == null) return;

            // Get the last spline added
            var spline = m_SplineContainer.Splines[m_SplineContainer.Splines.Count - 1];
            
            List<Vector3> worldPoints = new List<Vector3>();
            for (int i = 0; i < spline.Count; i++)
            {
                worldPoints.Add(m_SplineContainer.transform.TransformPoint((float3)spline[i].Position));
            }

            GameObject spawnedObject = Instantiate(m_PrefabToMove);
            SplineFollower follower = spawnedObject.GetComponent<SplineFollower>();
            if (follower == null)
            {
                follower = spawnedObject.AddComponent<SplineFollower>();
            }
            
            follower.InitAndStartMove(worldPoints);
        }
    }
}
