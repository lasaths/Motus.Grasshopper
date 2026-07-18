using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Urdf;
using Motus.Presets;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System;
using System.Linq;

namespace Motus.GH.Components;

internal static class CollisionNameUtil
{
    public static string Resolve(GH_Component owner, int nameIndex, string name, string defaultName)
    {
        if (nameIndex >= 0 && nameIndex < owner.Params.Input.Count && owner.Params.Input[nameIndex].SourceCount > 0)
            return string.IsNullOrWhiteSpace(name) ? $"{defaultName}_{ShortId(owner)}" : name;
        if (string.IsNullOrWhiteSpace(name) || string.Equals(name, defaultName, StringComparison.Ordinal))
            return $"{defaultName}_{ShortId(owner)}";
        return name;
    }

    private static string ShortId(GH_Component owner) => owner.InstanceGuid.ToString("N")[..4];
}

public sealed class MotusCollisionSphereComponent : MotusComponentBase
{
    private List<Mesh> _previewMeshes = new();
    private string? _previewKey;

    public MotusCollisionSphereComponent() : base("Motus Collision Sphere", "ColSph", "Sphere obstacle (meters)", "Collision", "sphere") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPointParameter("Center", "C", "Sphere center", GH_ParamAccess.item, Point3d.Origin);
        p.AddNumberParameter("Radius", "R", "Radius (m)", GH_ParamAccess.item, 0.1);
        p.AddTextParameter("Name", "N", "Obstacle name", GH_ParamAccess.item, "sphere");
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Object", "O", "Collision object", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var pt = Point3d.Origin;
        var r = 0.1;
        var name = "sphere";
        if (!da.GetData(0, ref pt) || !da.GetData(1, ref r)) return;
        da.GetData(2, ref name);
        name = CollisionNameUtil.Resolve(this, 2, name, "sphere");
        var frame = new Frame(pt.X, pt.Y, pt.Z);
        var obj = CollisionObject.Sphere(name, frame, r);
        var key = $"{pt.X:R},{pt.Y:R},{pt.Z:R}|{r:R}|{name}";
        if (_previewKey != key)
        {
            _previewKey = key;
            _previewMeshes = CollisionViewportPreview.MeshesFor(obj);
        }
        da.SetData(0, new CollisionObjectGoo(obj));
    }

    public override BoundingBox ClippingBox => CollisionViewportPreview.MeshesBoundingBox(_previewMeshes);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (!Locked) CollisionViewportPreview.DrawMeshes(args, _previewMeshes);
    }

    public override Guid ComponentGuid => new Guid("c1a2b3c4-d5e6-4789-a012-3456789abcde");
}

public sealed class MotusCollisionBoxComponent : MotusComponentBase
{
    private List<Mesh> _previewMeshes = new();
    private string? _previewKey;

    public MotusCollisionBoxComponent() : base("Motus Collision Box", "ColBox", "Axis-aligned box obstacle (half extents, m)", "Collision", "bounding-box") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPlaneParameter("Plane", "P", "Box center/orientation", GH_ParamAccess.item, Plane.WorldXY);
        p.AddNumberParameter("HalfX", "X", "Half extent X", GH_ParamAccess.item, 0.1);
        p.AddNumberParameter("HalfY", "Y", "Half extent Y", GH_ParamAccess.item, 0.1);
        p.AddNumberParameter("HalfZ", "Z", "Half extent Z", GH_ParamAccess.item, 0.1);
        p.AddTextParameter("Name", "N", "Obstacle name", GH_ParamAccess.item, "box");
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Object", "O", "Collision object", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var pl = Plane.WorldXY;
        var hx = 0.1; var hy = 0.1; var hz = 0.1;
        var name = "box";
        if (!da.GetData(0, ref pl) || !da.GetData(1, ref hx) || !da.GetData(2, ref hy) || !da.GetData(3, ref hz)) return;
        da.GetData(4, ref name);
        name = CollisionNameUtil.Resolve(this, 4, name, "box");
        var obj = CollisionObject.Box(name, FrameConversion.FromPlane(pl), hx, hy, hz);
        var key = $"{pl.OriginX:R},{pl.OriginY:R},{pl.OriginZ:R}|{hx:R},{hy:R},{hz:R}|{name}|{obj.ContentHash}";
        if (_previewKey != key)
        {
            _previewKey = key;
            _previewMeshes = CollisionViewportPreview.MeshesFor(obj);
        }
        da.SetData(0, new CollisionObjectGoo(obj));
    }

    public override BoundingBox ClippingBox => CollisionViewportPreview.MeshesBoundingBox(_previewMeshes);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (!Locked) CollisionViewportPreview.DrawMeshes(args, _previewMeshes);
    }

    public override Guid ComponentGuid => new Guid("d2b3c4d5-e6f7-4890-b123-456789abcdef");
}

public sealed class MotusCollisionSceneComponent : MotusComponentBase
{
    private List<Mesh> _previewMeshes = new();
    private string? _previewKey;
    private string? _srdfCachePath;
    private long _srdfCacheTicks;
    private CollisionScene? _srdfBaseScene;
    private List<PlanningGroup>? _srdfGroups;
    private List<string>? _srdfEndEffectors;

    public MotusCollisionSceneComponent() : base("Motus Collision Scene", "ColScene", "Merge collision objects; optional SRDF allowed pairs/groups", "Collision", "circles-three-plus") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Objects", "O", "Collision objects", GH_ParamAccess.list);
        p.AddTextParameter("Srdf", "S", "Optional SRDF file path (disable_collisions pairs)", GH_ParamAccess.item, "");
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddParameter(new Motus.GH.Params.Param_MotusCollisionScene(), "Scene", "Sc", "Collision scene", GH_ParamAccess.item);
        p.AddGenericParameter("Groups", "G", "Planning groups from SRDF (optional)", GH_ParamAccess.list);
        p.AddTextParameter("EndEffectors", "EE", "End-effector map from SRDF as name=parent_link entries", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var goos = new List<IGH_Goo>();
        if (!da.GetDataList(0, goos)) return;
        var objects = new List<CollisionObject>();
        foreach (var goo in goos)
        {
            if (GhExtract.TryCollisionObject(goo, out var co)) objects.Add(co);
            else AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Skipped non-collision input.");
        }
        var scene = new CollisionScene(objects);
        var srdfPath = "";
        da.GetData(1, ref srdfPath);
        var groups = new List<PlanningGroup>();
        var endEffectors = new List<string>();
        if (!string.IsNullOrWhiteSpace(srdfPath))
        {
            srdfPath = UrdfPathResolver.ResolveUrdfPath(srdfPath);
            if (!File.Exists(srdfPath))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"SRDF file not found: {srdfPath}");
            }
            else
            {
                try
                {
                    var ticks = File.GetLastWriteTimeUtc(srdfPath).Ticks;
                    if (_srdfCachePath != srdfPath || _srdfCacheTicks != ticks || _srdfGroups is null)
                    {
                        var doc = System.Xml.Linq.XDocument.Load(srdfPath);
                        var pairs = SrdfLoader.LoadAllowedPairs(doc);
                        _srdfBaseScene = SrdfLoader.MergeAllowedPairs(new CollisionScene(), pairs);
                        _srdfGroups = SrdfLoader.LoadGroups(doc).ToList();
                        _srdfEndEffectors = SrdfLoader.LoadEndEffectors(doc)
                            .Select(kv => $"{kv.Key}={kv.Value}")
                            .OrderBy(s => s, StringComparer.OrdinalIgnoreCase)
                            .ToList();
                        _srdfCachePath = srdfPath;
                        _srdfCacheTicks = ticks;
                    }

                    scene = SrdfLoader.MergeAllowedPairs(scene, _srdfBaseScene!.AllowedPairs);
                    groups = _srdfGroups!;
                    endEffectors = _srdfEndEffectors!;
                }
                catch (Exception ex)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"SRDF load failed: {ex.Message}");
                }
            }
        }
        da.SetData(0, new CollisionSceneGoo(scene));
        da.SetDataList(1, groups.Select(g => new PlanningGroupGoo(g)));
        da.SetDataList(2, endEffectors);

        var previewKey = string.Join("|", objects.Select(o => $"{o.Name}:{o.ContentHash}:{o.Pose.X:R},{o.Pose.Y:R},{o.Pose.Z:R}"));
        if (_previewKey != previewKey)
        {
            _previewKey = previewKey;
            _previewMeshes = CollisionViewportPreview.MeshesFor(scene);
        }
    }

    public override BoundingBox ClippingBox => CollisionViewportPreview.MeshesBoundingBox(_previewMeshes);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (!Locked) CollisionViewportPreview.DrawMeshes(args, _previewMeshes);
    }

    public override Guid ComponentGuid => new Guid("e3c4d5e6-f7a8-4901-c234-56789abcdef0");
}
