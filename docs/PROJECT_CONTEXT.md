# Project Context

Verified facts about MiniTrainWays. Check the code if anything here looks stale.

## Versions
- Unity 6000.3.8f1, URP 17.3.0 (`Assets/Settings/PC_RPAsset` + `PC_Renderer`, Forward+, SSAO feature).
- Splines 2.8.4, Mathematics, Input System 1.18.0 (Active Input Handling = Both; scripts use the old `Input` API).
- Test Framework 1.6.0. EditMode tests live in `Assets/Editor` (Assembly-CSharp-Editor already references NUnit).

## Scene wiring (`Assets/Scenes/SampleScene.unity`)
- `Spline`: `SplineContainer`, `SplineSampler`, `SplineTrack`, `MeshFilter`, `MeshRenderer` (material `Assets/Road/RoadSurface.mat`).
- `SplineMaker`: `RuntimeSplineGenerator`, referencing the `Spline` object's container and track.
- Main Camera looks straight down; the ground is raycast via `m_GroundLayer`.

## Scripts (`Assets/Scripts`, namespace `Unity.Splines` unless noted)
- `RuntimeSplineGenerator`: mouse drawing. Lerped stroke, knots every `m_KnotSpacing`, auto-finishes when the tip runs into a road. On finish it calls `SplineTrack.ConnectSplineToExistingRoads`, then `AutoDetectJunctions`. `m_RoadWidth` sets per-road width.
- `SplineTrack`: connections (T-splits, endpoint merges), junction detection, mesh building (road strips, intersection pads, two-road bend strips). It rebuilds itself in `LateUpdate` whenever a spline changes.
- `SplineSampler` (global namespace): road edges in a world-up frame (`TryGetSplineWidthEdges`).
- `JunctionBuilder` (global namespace): plain junction data.
- `SplineOverlay` (global namespace): Scene view overlay. Its "Remove Junction" button clears every junction.
- `SplineFollower`, `SplineObjectMover`: move objects along points/splines. Not part of road building.
- `SplineExtension/SplineEditorToolbarExtension`: spline selection helpers for the overlay.

## Non-obvious conventions
- `m_Width` and per-road widths are half-widths. Per-road width is spline embedded float data, key `roadWidth` (`GetRoadWidth`/`SetRoadWidth`). `SplitSplineOnKnot` does not copy it, so code copies it by hand.
- Strokes are drawn at container-local y = -2.
- Junction knots are linked; moving one drags the whole junction.
- Mesh UV0 is projective `(u·q, v·q, q)`. Vertex alpha 1 = paint lane markings, 0 = intersection pad.
- Road shader `Assets/Road/RoadSurface.shader`: asphalt is planar (object xz), markings come from UV0. Regenerate the textures and material with Tools > MiniTrainWays > Generate Road Surface (`Assets/Editor/RoadSurfaceGenerator.cs`).

## Testing
- `Assets/Editor/Tests/RoadNetworkTests.cs` (EditMode): checks coverage, overlap, normals, UVs and textures.
- Running tests through the Test Runner saves the open scene.
- Mouse input can't be sent over MCP. Simulate drags by calling the generator's private `StartNewSpline`/`AddKnotToSpline`/`FinishCurrentSpline` through reflection.

## Unity MCP quirks
- `execute_code` compiles as C# 6 (CodeDom). Write `UnityEngine.Object.FindFirstObjectByType<T>()` in full, and call spline extension methods statically (e.g. `SplineUtility.AddSpline(container)`).
- Calls made right after entering Play mode or a recompile can fail; retry once the editor is ready.
- Domain reloads clear non-serialized state, but `SplineTrack` re-detects junctions on enable.
