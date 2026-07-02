using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.Presets;
using Motus.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

public sealed class MotusCollisionSphereComponent : MotusComponentBase
{
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
        var frame = new Frame(pt.X, pt.Y, pt.Z);
        da.SetData(0, CollisionObject.Sphere(name, frame, r));
    }
    public override Guid ComponentGuid => new Guid("c1a2b3c4-d5e6-4789-a012-3456789abcde");
}

public sealed class MotusCollisionBoxComponent : MotusComponentBase
{
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
        da.SetData(0, CollisionObject.Box(name, FrameConversion.FromPlane(pl), hx, hy, hz));
    }
    public override Guid ComponentGuid => new Guid("d2b3c4d5-e6f7-4890-b123-456789abcdef");
}

public sealed class MotusCollisionSceneComponent : MotusComponentBase
{
    public MotusCollisionSceneComponent() : base("Motus Collision Scene", "ColScene", "Merge collision objects; optional SRDF allowed pairs", "Collision", "circles-three-plus") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Objects", "O", "Collision objects", GH_ParamAccess.list);
        p.AddTextParameter("Srdf", "S", "Optional SRDF file path (disable_collisions pairs)", GH_ParamAccess.item, "");
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Scene", "Sc", "Collision scene", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var raw = new List<object>();
        if (!da.GetDataList(0, raw)) return;
        var objects = new List<CollisionObject>();
        foreach (var o in raw)
        {
            if (o is CollisionObject co) objects.Add(co);
            else AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Skipped non-collision input.");
        }
        var scene = new CollisionScene(objects);
        var srdfPath = "";
        da.GetData(1, ref srdfPath);
        if (!string.IsNullOrWhiteSpace(srdfPath))
        {
            if (!File.Exists(srdfPath))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"SRDF file not found: {srdfPath}");
            }
            else
            {
                try
                {
                    var pairs = SrdfLoader.LoadAllowedPairs(srdfPath);
                    scene = SrdfLoader.MergeAllowedPairs(scene, pairs);
                }
                catch (Exception ex)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"SRDF load failed: {ex.Message}");
                }
            }
        }
        da.SetData(0, new CollisionSceneGoo(scene));
    }
    public override Guid ComponentGuid => new Guid("e3c4d5e6-f7a8-4901-c234-56789abcdef0");
}
