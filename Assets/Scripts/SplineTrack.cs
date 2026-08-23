using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;
using System.Collections.Generic;
//using UnityEditor.Splines;

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
        private List<List<JunctionBuilder.JunctionInfo>> m_JunctionGroups = new List<List<JunctionBuilder.JunctionInfo>>();
        
        private MeshFilter m_MeshFilter;
        private Mesh m_Mesh;

        private List<Vector3> m_Vertices = new List<Vector3>();
        private List<int> m_Triangles = new List<int>();
        private List<Vector2> m_UVs = new List<Vector2>();

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
        private void OnEnable()
        {
            //Spline.Changed += RebuildMesh;
            
        }
        private void OnDisable()
        {
            //Spline.Changed -= RebuildMesh;
        }
        
        // Start is called once before the first execution of Update after the MonoBehaviour is created
        void Start()
        {
            
        }

        // Update is called once per frame
        void Update()
        {
            // RebuildMesh removed from Update to improve performance
        }
        
        public void RebuildMesh()
        {
            if (splineContainer == null || splineContainer.Splines == null) return;
            
            m_Vertices.Clear();
            m_Triangles.Clear();
            m_UVs.Clear();
            
            Matrix4x4 worldToLocal = transform.worldToLocalMatrix;
            
            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                var spline = splineContainer.Splines[s];
                if (spline == null || spline.Count < 2) continue;
            
                int vertexOffset = m_Vertices.Count;
                float step = 1f / resolution;
                float splineLength = spline.GetLength();
            
                for (int i = 0; i <= resolution; i++)
                {
                    float t = i * step;
                    
                    // Get edge points using your existing logic
                    splineSampler.GetSplineWidthEdges(s, t, m_Width, out float3 leftEdge, out float3 rightEdge);
            
                    // Add vertices (convert to local space using cached matrix)
                    m_Vertices.Add(worldToLocal.MultiplyPoint3x4(leftEdge));
                    m_Vertices.Add(worldToLocal.MultiplyPoint3x4(rightEdge));
            
                    // UVs - Tiling based on spline length
                    float v = t * splineLength * m_TextureScale;
                    m_UVs.Add(new Vector2(0, v));
                    m_UVs.Add(new Vector2(1, v));
            
                    // Define triangles (except for the last segment)
                    if (i < resolution)
                    {
                        int start = vertexOffset + i * 2;
                        // Triangle 1
                        m_Triangles.Add(start);
                        m_Triangles.Add(start + 2);
                        m_Triangles.Add(start + 1);
                        
                        // Triangle 2
                        m_Triangles.Add(start + 1);
                        m_Triangles.Add(start + 2);
                        m_Triangles.Add(start + 3);
                    }
                }
                
            }
            
            foreach (var junctionGroup in m_JunctionGroups)
            {
                BuildJunctionMesh(junctionGroup, m_Vertices, m_Triangles, m_UVs);
            }
            
            if (m_Vertices.Count > 0)
            {
                m_Mesh.Clear();
                m_Mesh.SetVertices(m_Vertices);
                m_Mesh.SetTriangles(m_Triangles, 0);
                m_Mesh.SetUVs(0, m_UVs);
                m_Mesh.RecalculateNormals();
                m_Mesh.RecalculateBounds();
            }
            else
            {
                m_Mesh.Clear();
            }
        }

        private void BuildJunctionMesh(List<JunctionBuilder.JunctionInfo> junctions, List<Vector3> vertices, List<int> triangles, List<Vector2> uvs)
        {
            //if (junctions == null || junctions.Count < 2) return;

            List<Vector3> junctionEdges = new List<Vector3>();

            // 1. Sample junction edges for all junctions in this group
            foreach (var junction in junctions)
            {
                float t = junction.knotIndex == 0 ? 0f : 1f;
                splineSampler.GetSplineWidthEdges(junction.splineIndex, t, m_Width, out float3 l, out float3 r);
                
                junctionEdges.Add(transform.InverseTransformPoint(l));
                junctionEdges.Add(transform.InverseTransformPoint(r));
            }

            if (junctionEdges.Count == 0) return;

            // 2. Calculate center point
            Vector3 center = Vector3.zero;
            foreach (var p in junctionEdges)
                center += p;
            center /= junctionEdges.Count;

            // 3. Sort these points based on their direction from the center
            // We use the local Y axis as 'up' and sort on the XZ plane
            junctionEdges.Sort((a, b) =>
            {
                Vector3 dirA = a - center;
                Vector3 dirB = b - center;
                float angleA = Mathf.Atan2(dirA.z, dirA.x);
                float angleB = Mathf.Atan2(dirB.z, dirB.x);
                return angleA.CompareTo(angleB);
            });

            // 4. Wind triangles from the center point around each vert
            int centerIndex = vertices.Count;
            vertices.Add(center);
            uvs.Add(new Vector2(0.5f, 0.5f));

            int firstEdgeIndex = vertices.Count;
            for (int i = 0; i < junctionEdges.Count; i++)
            {
                vertices.Add(junctionEdges[i]);
                uvs.Add(new Vector2(0.5f + 0.5f * (junctionEdges[i].x - center.x), 0.5f + 0.5f * (junctionEdges[i].z - center.z)));
            }

            for (int i = 0; i < junctionEdges.Count; i++)
            {
                int current = firstEdgeIndex + i;
                int next = firstEdgeIndex + (i + 1) % junctionEdges.Count;

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
            Debug.Log("Junction Added with " + builder.junctions.Count + " points");
        }

        [ContextMenu("Auto Detect Junctions")]
        public void AutoDetectJunctions()
        {
            if (splineContainer == null || splineContainer.Splines == null) return;

            m_JunctionGroups.Clear();
            float threshold = 0.5f; // Slightly larger threshold for robustness
            
            Dictionary<int3, List<JunctionBuilder.JunctionInfo>> knotGroups = new Dictionary<int3, List<JunctionBuilder.JunctionInfo>>();

            for (int s = 0; s < splineContainer.Splines.Count; s++)
            {
                var spline = splineContainer.Splines[s];
                if (spline == null) continue;

                // For connections, we mostly care about start and end knots
                int[] knotsToCheck = new int[] { 0, spline.Count - 1 };
                if (spline.Count < 2) continue;

                foreach (int k in knotsToCheck)
                {
                    var knot = spline[k];
                    float3 pos = knot.Position;
                    int3 key = new int3(
                        Mathf.RoundToInt(pos.x / threshold), 
                        Mathf.RoundToInt(pos.y / threshold), 
                        Mathf.RoundToInt(pos.z / threshold)
                    );

                    if (!knotGroups.ContainsKey(key))
                        knotGroups[key] = new List<JunctionBuilder.JunctionInfo>();
                    
                    knotGroups[key].Add(new JunctionBuilder.JunctionInfo(s, k, spline, knot));
                }
            }

            foreach (var group in knotGroups.Values)
            {
                if (group.Count > 1)
                {
                    m_JunctionGroups.Add(group);
                }
            }

            RebuildMesh();
        }

    }
}
