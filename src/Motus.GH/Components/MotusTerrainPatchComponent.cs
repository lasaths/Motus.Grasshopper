using Grasshopper.Kernel;
using Motus.GH.Planning;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>Soft outdoor heightfield mesh for Motus Walk <c>Tn</c> (meters, Z-up).</summary>
public sealed class MotusTerrainPatchComponent : MotusComponentBase
{
    public static readonly Guid Id = new("86e87c03-366b-4de3-9448-3b154cd28f24");

    public MotusTerrainPatchComponent()
        : base(
            "Motus Terrain Patch",
            "Ground",
            "Outdoor-style heightfield mesh (m) for Motus Walk Terrain — gentle hills, wire to Tn.",
            "Model",
            "polygon")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPointParameter("Origin", "O", "Patch center (m)", GH_ParamAccess.item, new Point3d(0.22, 0, 0));
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("SizeX", "Sx", "Full width X (m)", GH_ParamAccess.item, 1.2);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("SizeY", "Sy", "Full depth Y (m)", GH_ParamAccess.item, 1.0);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Amp", "A", "Hill amplitude (m) — keep below Walk Lift", GH_ParamAccess.item, 0.04);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddMeshParameter("Mesh", "M", "Outdoor ground mesh (m)", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var origin = new Point3d(0.22, 0, 0);
        var sx = 1.2;
        var sy = 1.0;
        var amp = 0.04;
        da.GetData(0, ref origin);
        da.GetData(1, ref sx);
        da.GetData(2, ref sy);
        da.GetData(3, ref amp);
        if (!origin.IsValid || !double.IsFinite(sx) || !double.IsFinite(sy) || !double.IsFinite(amp))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Origin / Size / Amp must be finite.");
            return;
        }

        da.SetData(0, OutdoorTerrainMesh.Build(origin, sx, sy, amp));
    }

    public override Guid ComponentGuid => Id;
}
