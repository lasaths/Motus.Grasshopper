using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Motus.Core;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Rhino.Geometry;
using System.Runtime.CompilerServices;

namespace Motus.GH.Components;

public sealed class MotusCollisionMeshComponent : MotusComponentBase
{
    private List<Mesh> _previewMeshes = new();
    private string? _previewKey;

    public MotusCollisionMeshComponent() : base("Motus Collision Mesh", "ColMesh", "Mesh or Brep obstacle (meters)", "Collision", "polygon") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGeometryParameter("Geometry", "G", "Triangle mesh or Brep obstacle", GH_ParamAccess.item);
        p.AddPlaneParameter("Plane", "P", "Geometry pose (origin = local origin)", GH_ParamAccess.item, Plane.WorldXY);
        p.AddTextParameter("Name", "N", "Obstacle name", GH_ParamAccess.item, "mesh");
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Object", "O", "Collision object", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        IGH_GeometricGoo? geo = null;
        var pl = Plane.WorldXY;
        var name = "mesh";
        if (!da.GetData(0, ref geo) || geo is null) return;
        da.GetData(1, ref pl);
        da.GetData(2, ref name);
        name = CollisionNameUtil.Resolve(this, 2, name, "mesh");

        CollisionObject? obj = null;
        var triangleCount = 0;
        Mesh? sourceMesh = null;
        if (geo is GH_Mesh ghm && ghm.Value is { IsValid: true } mesh)
        {
            sourceMesh = mesh;
            obj = CollisionMeshBuilder.FromMesh(mesh, pl, name, out triangleCount);
        }
        else if (geo is GH_Brep ghb && ghb.Value is { IsValid: true } brep)
            obj = CollisionMeshBuilder.FromBrep(brep, pl, name, out triangleCount);

        if (obj is null)
        {
            _previewMeshes = [];
            _previewKey = null;
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Geometry must be a valid Mesh or Brep.");
            return;
        }

        if (triangleCount > CollisionMeshBuilder.DenseTriangleWarnThreshold)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"Dense collision mesh ({triangleCount:N0} tris). Decimate or use ColBox/ColSphere for faster planning.");
        }

        var key = $"{geo.GetType().Name}|{RuntimeHelpers.GetHashCode(geo)}|{pl.OriginX:R},{pl.OriginY:R},{pl.OriginZ:R}|{name}|{obj.ContentHash}";
        if (_previewKey != key)
        {
            _previewKey = key;
            // Prefer source Rhino mesh for preview (avoid Motus→Rhino reconversion).
            if (sourceMesh is not null)
            {
                var preview = sourceMesh.DuplicateMesh();
                if (!pl.Origin.Equals(Point3d.Origin) ||
                    !pl.XAxis.Equals(Vector3d.XAxis) ||
                    !pl.YAxis.Equals(Vector3d.YAxis))
                    preview.Transform(Transform.PlaneToPlane(Plane.WorldXY, pl));
                _previewMeshes = [preview];
            }
            else
                _previewMeshes = CollisionViewportPreview.MeshesFor(obj);
        }

        da.SetData(0, new CollisionObjectGoo(obj));
    }

    public override BoundingBox ClippingBox => CollisionViewportPreview.MeshesBoundingBox(_previewMeshes);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (!Locked) CollisionViewportPreview.DrawMeshes(args, _previewMeshes);
    }

    public override Guid ComponentGuid => new Guid("f4d5e6f7-a8b9-4012-d345-6789abcdef01");
}
