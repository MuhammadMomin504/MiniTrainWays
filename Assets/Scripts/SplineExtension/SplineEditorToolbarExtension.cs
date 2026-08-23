using System.Collections.Generic;
using System.Linq;
using UnityEditor.Overlays;
using UnityEngine;
using UnityEngine.Splines;
using UnityEngine.UIElements;

namespace UnityEditor.Splines
{
    public static class SplineEditorToolbarExtension
    {
        public struct SelectedSplineElementInfo
        {
            public Object target;
            public int targetIndex;
            public int knotIndex;

            public SelectedSplineElementInfo(Object target, int targetIndex, int knotIndex)
            {
                this.target = target;
                this.targetIndex = targetIndex;
                this.knotIndex = knotIndex;
            }
            
        }

        public static bool HasSelection()
        {
            return SplineSelection.HasActiveSplineSelection();
        }
        
        public static List<SelectedSplineElementInfo> GetSelection()
        {
            List<SelectableSplineElement> elements = SplineSelection.selection;

            List<SelectedSplineElementInfo> infos = new List<SelectedSplineElementInfo>();

            foreach (SelectableSplineElement element in elements)
            {
                infos.Add(new SelectedSplineElementInfo(element.target, element.targetIndex, element.knotIndex));
            }
            
            return infos;
        }
    }
   
}
