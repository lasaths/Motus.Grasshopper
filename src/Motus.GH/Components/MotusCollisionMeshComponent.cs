using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

public sealed class MotusCollisionMeshComponent : MotusComponentBase
{
    public MotusCollisionMeshComponent() : base("Motus Collision Mesh", "ColMesh", "Rhino mesh obstacle (meters)", "Collision", "mesh") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddMeshParameter("Mesh", "M", "Triangle mesh obstacle", GH_ParamAccess.item);
        p.AddPlaneParameter("Plane", "P", "Mesh pose (origin = mesh local origin)", GH_ParamAccess.item, Plane.WorldXY);
        p.AddTextParameter("Name", "N", "Obstacle name", GH_ParamAccess.item, "mesh");
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Object", "O", "Collision object", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        Mesh? mesh = null;
        var pl = Plane.WorldXY;
        var name = "mesh";
        if (!da.GetData(0, ref mesh) || mesh is null || !mesh.IsValid) return;
        da.GetData(1, ref pl);
        da.GetData(2, ref name);

        var vertices = new List<double[]>(mesh.Vertices.Count);
        foreach (var v in mesh.Vertices)
            vertices.Add(new[] { (double)v.X, (double)v.Y, (double)v.Z });

        var indices = new List<int>(mesh.Faces.Count * 3);
        foreach (var face in mesh.Faces)
        {
            if (face.IsTriangle)
            {
                indices.Add(face.A);
                indices.Add(face.B);
                indices.Add(face.C);
            }
            else
            {
                indices.Add(face.A);
                indices.Add(face.B);
                indices.Add(face.C);
                indices.Add(face.A);
                indices.Add(face.C);
                indices.Add(face.D);
            }
        }

        if (indices.Count < 3)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Mesh has no faces.");
            return;
        }

        da.SetData(0, CollisionObject.Mesh(name, FrameConversion.FromPlane(pl), vertices, indices));
    }

    public override Guid ComponentGuid => new Guid("f4d5e6f7-a8b9-4012-d345-6789abcdef01");
}
