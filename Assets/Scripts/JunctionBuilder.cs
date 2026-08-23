using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

    public class JunctionBuilder
    {

        public struct JunctionInfo
        {
            public int splineIndex;
            public int knotIndex;
            public Spline spline;
            public BezierKnot knot;

            public JunctionInfo(int splineIndex, int knotIndex, Spline spline, BezierKnot knot)
            {
                this.splineIndex = splineIndex;
                this.knotIndex = knotIndex;
                this.spline = spline;
                this.knot = knot;
            }
            
        }
        
        public List<JunctionInfo> junctions;

        public void AddJunction(int splineIndex, int knotIndex, Spline spline, BezierKnot knot)
        {
            if (junctions == null)
                junctions = new List<JunctionInfo>();
            junctions.Add(new JunctionInfo(splineIndex, knotIndex, spline, knot));
        }

        internal IEnumerable<JunctionInfo> GetJunctions()
        {
            return junctions;
        }
        
    }

