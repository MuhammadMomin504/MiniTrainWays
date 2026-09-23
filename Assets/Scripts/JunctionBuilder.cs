using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

    public class JunctionBuilder
    {

        public struct JunctionInfo
        {
            public int splineIndex;
            public int knotIndex;
            public float t;
            public Spline spline;
            public BezierKnot knot;

            public JunctionInfo(int splineIndex, int knotIndex, float t, Spline spline, BezierKnot knot)
            {
                this.splineIndex = splineIndex;
                this.knotIndex = knotIndex;
                this.t = t;
                this.spline = spline;
                this.knot = knot;
            }
            
        }
        
        public List<JunctionInfo> junctions;

        public void AddJunction(int splineIndex, int knotIndex, float t, Spline spline, BezierKnot knot)
        {
            if (junctions == null)
                junctions = new List<JunctionInfo>();
            junctions.Add(new JunctionInfo(splineIndex, knotIndex, t, spline, knot));
        }

        internal IEnumerable<JunctionInfo> GetJunctions()
        {
            return junctions;
        }
        
    }

