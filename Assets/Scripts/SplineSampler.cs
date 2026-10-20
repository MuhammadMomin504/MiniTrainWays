using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine;
using UnityEngine.Splines;

[ExecuteInEditMode()]
public class SplineSampler : MonoBehaviour
{
    [SerializeField] private SplineContainer splineContainer = default;
    
    [SerializeField] private int splineIndex = 0;
    [SerializeField] [Range(0f, 1f)] private float time = 0f;

    [SerializeField] private float m_Width = 0.1f;
    [SerializeField] private int m_NumSamples = 10;
    
    
    private float3 position;
    private float3 tangent;
    private float3 upVector;
    
    private float3 p1;
    private float3 p2;
    
    
    
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update() 
    {
        // if (splineContainer != null)
        // {
        //     // Call Evaluate using the splineIndex and time defined in your class
        //     // This will populate the position, tangent, and upVector fields
        //     time += Time.deltaTime * 0.25f;
        //     // if(time > 1f)
        //     //     time = 0f;
        //     splineContainer.Evaluate(splineIndex, time, out position, out tangent, out upVector);
        //     
        //     float3 right = math.cross(tangent, upVector);
        //     p1 = position + (right * m_Width);
        //     p2 = position + (-right * m_Width);
        //
        //     // Optionally, if you want to see the result in the Scene view:
        //     // Debug.DrawRay(position, tangent, Color.blue);
        //     // Debug.DrawRay(position, upVector, Color.green);
        // }
    }
    
    public void GetSplineWidthEdges(int index, float t, float width, out float3 leftEdge, out float3 rightEdge)
    {
        TryGetSplineWidthEdges(index, t, width, out _, out _, out leftEdge, out rightEdge);
    }

    /// <summary>
    /// Constant-width road frame. Uses world-up so a join cannot flip the ribbon
    /// when spline up/tangent become unstable.
    /// </summary>
    public bool TryGetSplineWidthEdges(
        int index,
        float t,
        float width,
        out float3 position,
        out float3 tangent,
        out float3 leftEdge,
        out float3 rightEdge)
    {
        position = tangent = leftEdge = rightEdge = float3.zero;
        if (splineContainer == null || index < 0 || index >= splineContainer.Splines.Count)
            return false;

        t = math.saturate(t);
        splineContainer.Evaluate(index, t, out position, out float3 tan, out _);

        if (math.lengthsq(tan) < 1e-8f)
        {
            float t2 = math.saturate(t < 0.5f ? t + 0.02f : t - 0.02f);
            splineContainer.Evaluate(index, t2, out float3 pos2, out _, out _);
            tan = t2 >= t ? pos2 - position : position - pos2;
        }

        tangent = math.normalizesafe(tan);
        if (math.lengthsq(tangent) < 1e-8f)
            return false;

        float3 worldUp = new float3(0f, 1f, 0f);
        float3 right = math.cross(tangent, worldUp);
        if (math.lengthsq(right) < 1e-8f)
            right = math.cross(tangent, new float3(0f, 0f, 1f));
        right = math.normalizesafe(right);

        leftEdge = position + right * width;
        rightEdge = position - right * width;
        return true;
    }

    private void OnDrawGizmos()
    {
        // Handles.matrix = transform.localToWorldMatrix;
        // Handles.SphereHandleCap(0, p1, Quaternion.identity, 0.1f, EventType.Repaint);
        // Handles.SphereHandleCap(0, p2, Quaternion.identity, 0.1f, EventType.Repaint);

        var samples = SamplePointsAroundMesh(m_NumSamples);
        Gizmos.color = Color.yellow;
        // foreach (var sample in samples)
        // {
        //     Gizmos.DrawSphere(sample, 0.05f);
        // }
    }

    /// <summary>
    /// Samples a number of points along all splines in the container.
    /// Each sample includes the left and right edge points based on m_Width.
    /// </summary>
    /// <param name="numPointsPerSpline">Number of samples per spline.</param>
    /// <returns>A list of sampled points (positions).</returns>
    public List<Vector3> SamplePointsAroundMesh(int numPointsPerSpline)
    {
        List<Vector3> sampledPoints = new List<Vector3>();

        if (splineContainer == null)
            return sampledPoints;

        for (int i = 0; i < splineContainer.Splines.Count; i++)
        {
            float step = 1f / (numPointsPerSpline - 1);
            for (int j = 0; j < numPointsPerSpline; j++)
            {
                float t = j * step;
                GetSplineWidthEdges(i, t, m_Width, out float3 left, out float3 right);
                
                // Add left and right points to represent "around the mesh"
                sampledPoints.Add(left);
                sampledPoints.Add(right);
            }
        }

        return sampledPoints;
    }
}
