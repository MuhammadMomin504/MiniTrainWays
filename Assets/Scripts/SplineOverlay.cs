using System.Collections.Generic;
using System.Linq;
using Unity.Splines;
using UnityEditor;
using UnityEditor.Overlays;
using UnityEditor.Splines;
using UnityEngine;
using UnityEngine.Splines;
using UnityEngine.UIElements;

[Overlay(typeof(SceneView), "Spline Selection Info", true)]
public class SplineOverlay : Overlay
{
    
    private VisualElement m_Root;
    private Label m_SelectionLabel;
    private List<Vector3> m_JunctionPositions = new List<Vector3>();

    public override VisualElement CreatePanelContent()
    {
        m_Root = new VisualElement() { name = "Spline Selection Root" };
        m_Root.style.paddingTop = 5;
        m_Root.style.paddingBottom = 5;
        m_Root.style.paddingLeft = 5;
        m_Root.style.paddingRight = 5;
        m_Root.style.minWidth = 150;

        m_SelectionLabel = new Label("No Spline Selected");
        m_Root.Add(m_SelectionLabel);

        Button buildJunctionButton = new Button(OnBuildJunction) { text = "Build Junction" };
        m_Root.Add(buildJunctionButton);

        Button removeJunctionButton = new Button(OnRemoveJunction) { text = "Remove Junction" };
        m_Root.Add(removeJunctionButton);

        UpdateSelectionInfo();

        return m_Root;
    }

    public override void OnCreated()
    {
        SplineSelection.changed += OnSelectionChanged;
        SceneView.duringSceneGui += OnSceneGUI;
    }

    public override void OnWillBeDestroyed()
    {
        SplineSelection.changed -= OnSelectionChanged;
        SceneView.duringSceneGui -= OnSceneGUI;
    }

    private void OnSceneGUI(SceneView sceneView)
    {
        Handles.color = Color.blue;
        foreach (var pos in m_JunctionPositions)
        {
            // Handles.DrawWireDisc(pos, Vector3.up, 0.5f);
            // Handles.DrawSolidDisc(pos, Vector3.up, 0.1f);
        }
    }

    private void OnSelectionChanged()
    {
        UpdateSelectionInfo();
    }

    private void ClearSelectionInfo()
    {
        if (m_SelectionLabel != null)
            m_SelectionLabel.text = "No Spline Selected";
    }

    private void UpdateSelectionInfo()
    {
        if (m_SelectionLabel == null)
            return;

        if (!SplineEditorToolbarExtension.HasSelection())
        {
            ClearSelectionInfo();
            //return;
        }

        var selection = SplineEditorToolbarExtension.GetSelection();
        if (selection.Count == 0)
        {
            ClearSelectionInfo();
            return;
        }

        string info = "Selected Splines:\n";
        HashSet<string> seen = new HashSet<string>();
        foreach (var element in selection)
        {
            string entry = $"- {element.target.name} [Spline {element.targetIndex}, Knot {element.knotIndex}]";
            if (seen.Add(entry))
            {
                info += entry + "\n";
            }
        }

        m_SelectionLabel.text = info.TrimEnd();
    }
        
    private void OnBuildJunction()
    {
        List<SplineEditorToolbarExtension.SelectedSplineElementInfo> selection = SplineEditorToolbarExtension.GetSelection();

        JunctionBuilder builder = new JunctionBuilder();
        foreach (SplineEditorToolbarExtension.SelectedSplineElementInfo item in selection)
        {
            SplineContainer container = (SplineContainer)item.target;
            Spline spline = container.Splines[item.targetIndex];
            BezierKnot knot = spline.Knots.ElementAt(item.knotIndex);
            
            // Calculate t for the knot
            float t = (float)item.knotIndex / (spline.Count - 1);
            builder.AddJunction(item.targetIndex, item.knotIndex, t, spline, knot);

            // Store world position for overlay
            Vector3 worldPos = container.transform.TransformPoint(knot.Position);
            m_JunctionPositions.Add(worldPos);
        }

        if (Selection.activeGameObject != null)
        {
            SplineTrack track = Selection.activeGameObject.GetComponent<SplineTrack>();
            if (track != null)
            {
                track.AddJunction(builder);
            }
            else
            {
                Debug.LogWarning("Selected GameObject does not have a SplineTrack component.");
            }
        }
    }

    private void OnRemoveJunction()
    {
        m_JunctionPositions.Clear();
        if (Selection.activeGameObject != null)
        {
            SplineTrack track = Selection.activeGameObject.GetComponent<SplineTrack>();
            if (track != null)
            {
                track.ClearJunctions();
                track.RebuildMesh();
            }
        }
    }
    
   
}
