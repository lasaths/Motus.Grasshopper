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

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Motus Robot (Serial Chain or URDF)", GH_ParamAccess.item);
        p.AddIntegerParameter("Count", "N", $"Max TCP samples (default {DefaultCount}, max {MaxCount})", GH_ParamAccess.item, DefaultCount);
        p[p.ParamCount - 1].Optional = true;
        p.AddIntegerParameter("Seed", "Seed", "Reserved (Halton sequence; currently unused)", GH_ParamAccess.item, 0);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddPointParameter("Points", "Pts", "Sampled TCP points in base frame", GH_ParamAccess.list);

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
            var tree = goo.Tree;
            SerialJointChain? chain = goo.Chain;
            goo.EnsureChainFromPath();
            chain ??= goo.Chain;

            if (tree is null && !string.IsNullOrWhiteSpace(goo.UrdfSourcePath))
            {
                tree = Motus.Presets.UrdfRobotLoader.LoadTree(goo.UrdfSourcePath);
                goo.Tree = tree;
            }

            if (tree is null && chain is not null)
            {
                // rebuild a length-less tree from chain origins for reach (use joint limits from preset)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                    "Robot has no KinematicTree. Use Motus Serial Chain or a URDF Motus Robot.");
                return;
            }

            if (tree is null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Robot has no kinematic tree.");
                return;
            }

            var tipName = "tool0";
            int tipIdx;
            try
            {
                tipIdx = tree.IndexOfLink(tipName);
            }
            catch
            {
                tipIdx = tree.Links.Count - 1;
            }

            var lower = new double[tree.DriverCount];
            var upper = new double[tree.DriverCount];
            var limits = goo.Value.Preset.JointLimits;
            for (var i = 0; i < tree.DriverCount; i++)
            {
                var j = tree.Joints[tree.DriverJointIndices[i]];
                if (i < limits.Count)
                {
                    lower[i] = limits[i].MinRadians;
                    upper[i] = limits[i].MaxRadians;
                }
                else
                {
                    lower[i] = j.Lower;
                    upper[i] = j.Upper;
                }
            }

            var fk = new TreeForwardKinematics(tree);
            var xyz = new double[count * 3];
            var n = ReachSampling.FillTcpPointsInto(fk, tipIdx, lower, upper, xyz, count);

            var baseM = Transforms.FromFrame(goo.EffectiveBase().Frame);
            var pts = new List<Point3d>(n);
            for (var i = 0; i < n; i++)
            {
                var o = i * 3;
                var lx = xyz[o];
                var ly = xyz[o + 1];
                var lz = xyz[o + 2];
                // column-major? Motus mats are row-major 4x4 flat: [0..3 row0], transform p' = R*p + t
                var wx = baseM[0] * lx + baseM[1] * ly + baseM[2] * lz + baseM[3];
                var wy = baseM[4] * lx + baseM[5] * ly + baseM[6] * lz + baseM[7];
                var wz = baseM[8] * lx + baseM[9] * ly + baseM[10] * lz + baseM[11];
                pts.Add(new Point3d(wx, wy, wz));
            }

            da.SetDataList(0, pts);
        }
        catch (Exception ex)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new("a1b2c3d4-5e6f-7081-92a3-b4c5d6e7f809");
}
