using Grasshopper.Kernel;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>Stratified TCP reach samples from a Motus Robot (Serial Chain or URDF).</summary>
public sealed class MotusReachSamplesComponent : MotusComponentBase
{
    private const int DefaultCount = 512;
    private const int MaxCount = 512;

    public MotusReachSamplesComponent()
        : base(
            "Motus Reach Samples",
            "Reach",
            "Sample TCP points inside joint limits (capped, stratified). Overlay on structure geometry in Rhino.",
            "Preview",
            "circles-three-plus")
    {
    }

    public override GH_Exposure Exposure => GH_Exposure.secondary;

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Motus Robot (Serial Chain or URDF)", GH_ParamAccess.item);
        p.AddIntegerParameter("Count", "N", $"Max TCP samples (default {DefaultCount}, max {MaxCount})", GH_ParamAccess.item, DefaultCount);
        p[p.ParamCount - 1].Optional = true;
        p.AddIntegerParameter("Seed", "Seed", "Deterministic sample seed (same seed gives the same reach cloud)", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddPointParameter("Points", "Pts", "Sampled selected TCP points in world coordinates (includes Base and Tool); not a reachability guarantee", GH_ParamAccess.list);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        RobotModelGoo? goo = null;
        if (!da.GetData(0, ref goo) || goo?.Value is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Wire a Motus Robot (Rb).");
            return;
        }

        var count = DefaultCount;
        da.GetData(1, ref count);
        count = Math.Clamp(count, 0, MaxCount);
        if (count == 0)
        {
            da.SetDataList(0, Array.Empty<Point3d>());
            return;
        }

        try
        {
            goo.EnsureChainFromPath();
            if (goo.Chain is not { } chain)
                throw new ArgumentException("Reach requires a serial tip chain. Use a serial Robot or Robot From Description.");
            var seed = 0;
            da.GetData(2, ref seed);
            var frames = SerialReachSamples.Sample(goo.EffectiveModel(), chain, count, seed);
            var pts = frames.Select(f => new Point3d(f.X, f.Y, f.Z)).ToList();
            da.SetDataList(0, pts);
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new("a1b2c3d4-5e6f-7081-92a3-b4c5d6e7f809");
}
