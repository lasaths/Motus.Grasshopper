using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>Many box obstacles from a plane list (tower / pallet authoring).</summary>
public sealed class MotusCollisionBoxesComponent : MotusComponentBase
{
    private List<Mesh> _previewMeshes = new();
    private string? _previewKey;

    public MotusCollisionBoxesComponent()
        : base("Motus Collision Boxes", "Boxes", "Box obstacles from a plane list (half extents, m)", "Collision", "bounding-box") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Next: O->Motus Collision Scene / Motus Pick Place Objects",
        "Wire: Plane list (one per brick)",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPlaneParameter("Planes", "P", "Box centers; plane XYZ = box XYZ", GH_ParamAccess.list);
        p.AddNumberParameter("HalfX", "X", "Half extent X (m)", GH_ParamAccess.item, 0.04);
        p.AddNumberParameter("HalfY", "Y", "Half extent Y (m)", GH_ParamAccess.item, 0.02);
        p.AddNumberParameter("HalfZ", "Z", "Half extent Z (m)", GH_ParamAccess.item, 0.01);
        p.AddTextParameter("Prefix", "N", "Name prefix → N00, N01, …", GH_ParamAccess.item, "b");
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Objects", "O", "Collision objects", GH_ParamAccess.list);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var planes = new List<Plane>();
        if (!da.GetDataList(0, planes) || planes.Count == 0) return;
        var hx = 0.04;
        var hy = 0.02;
        var hz = 0.01;
        var prefix = "b";
        da.GetData(1, ref hx);
        da.GetData(2, ref hy);
        da.GetData(3, ref hz);
        da.GetData(4, ref prefix);
        if (string.IsNullOrWhiteSpace(prefix)) prefix = "b";

        var objects = new List<CollisionObjectGoo>(planes.Count);
        for (var i = 0; i < planes.Count; i++)
        {
            var pl = planes[i];
            if (!pl.IsValid) continue;
            var name = $"{prefix}{i:D2}";
            var obj = CollisionObject.Box(name, FrameConversion.FromPlanePlate(pl), hx, hy, hz);
            objects.Add(new CollisionObjectGoo(obj));
        }

        var key = string.Join("|", objects.Select(o => o.Value is null ? "" : $"{o.Value.Name}:{o.Value.ContentHash}"));
        if (_previewKey != key)
        {
            foreach (var mesh in _previewMeshes)
                mesh.Dispose();
            _previewKey = key;
            _previewMeshes = new List<Mesh>(objects.Count);
            foreach (var goo in objects)
            {
                if (goo.Value is { } obj)
                    _previewMeshes.AddRange(CollisionViewportPreview.MeshesFor(obj));
            }
        }

        da.SetDataList(0, objects);
    }

    public override BoundingBox ClippingBox => CollisionViewportPreview.MeshesBoundingBox(_previewMeshes);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (!Locked) CollisionViewportPreview.DrawMeshes(args, _previewMeshes);
    }

    public override Guid ComponentGuid => new("a4b5c6d7-e8f9-4012-b345-6789abcdef01");
}

/// <summary>Expand grasp/place/object lists into Motus Move segments (pick-place cycle × N).</summary>
public sealed class MotusPickPlaceComponent : MotusComponentBase
{
    public MotusPickPlaceComponent()
        : base(
            "Motus Pick Place",
            "PickPlace",
            "Expand grasp/place/object lists into Motus Move segments (LIN/SET/Attach/Detach cycles)",
            "Plan",
            "line-segments") { }

    protected override IReadOnlyList<string> AiKeywords { get; } =
    [
        "Next: Seg->Motus Program Segments",
        "Wire: Grasp + Place plane lists; Objects from Motus Boxes",
        "Note: Open/Close jaw widths (m); Approach = world +Z hover",
    ];

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPlaneParameter("Grasp", "G", "Grasp TCP planes (visit order)", GH_ParamAccess.list);
        p.AddPlaneParameter("Place", "Pl", "Place TCP planes (same count as Grasp)", GH_ParamAccess.list);
        p.AddGenericParameter("Objects", "O", "Collision objects to attach (same count)", GH_ParamAccess.list);
        p.AddNumberParameter("Approach", "Az", "Hover height above grasp/place along world +Z (m)", GH_ParamAccess.item, 0.08);
        p.AddNumberParameter("Open", "Wopen", "Open jaw width (m)", GH_ParamAccess.item, 0.085);
        p.AddNumberParameter("Close", "Wclose", "Close jaw width (m) — brick short side", GH_ParamAccess.item, 0.04);
        p.AddNumberParameter("Step", "St", "LIN step (m)", GH_ParamAccess.item, 0.005);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusSegment(), "Segments", "Seg", "Motion segments for Motus Program", GH_ParamAccess.list);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var graspPlanes = new List<Plane>();
        var placePlanes = new List<Plane>();
        var rawObjects = new List<IGH_Goo>();
        if (!da.GetDataList(0, graspPlanes) || !da.GetDataList(1, placePlanes) || !da.GetDataList(2, rawObjects))
            return;

        var approach = 0.08;
        var openW = 0.085;
        var closeW = 0.04;
        var step = 0.005;
        da.GetData(3, ref approach);
        da.GetData(4, ref openW);
        da.GetData(5, ref closeW);
        da.GetData(6, ref step);

        if (graspPlanes.Count != placePlanes.Count || graspPlanes.Count != rawObjects.Count)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                $"Grasp ({graspPlanes.Count}), Place ({placePlanes.Count}), Objects ({rawObjects.Count}) must match.");
            return;
        }

        if (graspPlanes.Count == 0) return;
        if (approach < 0 || step <= 0 || openW < 0 || closeW < 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Approach/Step/Open/Close must be non-negative (Step > 0).");
            return;
        }

        var grasps = new List<CartesianPose>(graspPlanes.Count);
        var places = new List<CartesianPose>(placePlanes.Count);
        var objects = new List<CollisionObject>(rawObjects.Count);
        for (var i = 0; i < graspPlanes.Count; i++)
        {
            if (!graspPlanes[i].IsValid || !placePlanes[i].IsValid)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Invalid plane at index {i}.");
                return;
            }

            grasps.Add(new CartesianPose(FrameConversion.FromPlane(graspPlanes[i])));
            places.Add(new CartesianPose(FrameConversion.FromPlane(placePlanes[i])));
            if (!GhExtract.TryCollisionObject(rawObjects[i], out var co))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Objects[{i}] must be a Motus collision object.");
                return;
            }

            objects.Add(co);
        }

        var open = new EndEffectorState(new Dictionary<string, double> { ["width"] = openW });
        var close = new EndEffectorState(new Dictionary<string, double> { ["width"] = closeW });
        IReadOnlyList<MotionSegment> segments;
        try
        {
            segments = PickPlaceCycle.ExpandMany(grasps, places, objects, approach, open, close, step);
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
            return;
        }

        da.SetDataList(0, segments.Select(s => new MotionSegmentGoo(s)));
    }

    public override Guid ComponentGuid => new("b5c6d7e8-f9a0-4123-c456-789abcdef012");
}
