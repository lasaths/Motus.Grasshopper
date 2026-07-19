using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>Parametric serial (optional rail) arm from length list → same Robot goo as Motus Robot.</summary>
public sealed class MotusSerialChainComponent : RobotSourceComponentBase
{
    private long _lastFingerprint;

    public MotusSerialChainComponent()
        : base(
            "Motus Serial Chain",
            "Serial",
            "Build a parametric serial robot from link lengths (optional rail). Outputs the same Robot as Motus Robot.",
            "path")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Lengths", "L", "Link lengths (m). With Rail: first = stroke, rest = arm.", GH_ParamAccess.list);
        p.AddPlaneParameter("Base", "B", "Optional base frame", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Home", "Q", "Optional home joint values (driver order)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("Rail", "Rail", "First length is prismatic stroke (+Z); rest revolute", GH_ParamAccess.item, false);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Types", "Types", "Optional R/P per joint (ignored when Rail)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("TCP", "TCP", "Optional tip tool frame in last-link frame", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Robot model (same as Motus Robot)", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var lengths = new List<double>();
        if (!da.GetDataList(0, lengths) || lengths.Count == 0)
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Wire Lengths (L) — one number per joint/link.");
            return;
        }

        var basePl = Plane.Unset;
        da.GetData(1, ref basePl);
        var home = new List<double>();
        da.GetDataList(2, home);
        var rail = false;
        da.GetData(3, ref rail);
        var types = new List<string>();
        da.GetDataList(4, types);
        var tcpPl = Plane.Unset;
        da.GetData(5, ref tcpPl);

        if (!rail && types.Count > 0 && types.Count != lengths.Count)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                $"Types count ({types.Count}) ≠ Lengths ({lengths.Count}).");
        }

        try
        {
            var tree = SerialKinematicTrees.FromLengths(
                lengths,
                rail,
                "serial_chain",
                types.Count > 0 && !rail ? types : null);

            if (tree.Fingerprint == _lastFingerprint && _previewMeshes.Count > 0)
            {
                // still emit goo each solve (GH needs output) — rebuild is cheap enough; fingerprint skips mesh rebake via ApplyPreview key
            }

            _lastFingerprint = tree.Fingerprint;
            var tip = tree.ExtractSerialTip("base_link", "tool0");
            var limits = new List<JointLimit>(tree.DriverCount);
            for (var i = 0; i < tree.DriverCount; i++)
            {
                var j = tree.Joints[tree.DriverJointIndices[i]];
                var vel = j.Velocity ?? Math.PI;
                limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
            }

            var toolFrame = ToolFrame.Identity;
            if (tcpPl.IsValid)
                toolFrame = new ToolFrame(FrameConversion.FromPlane(tcpPl), "tcp");
            else if (tip.TipToolOffset is { } tipOff)
                toolFrame = new ToolFrame(tipOff, "tool0");

            var preset = new RobotPreset
            {
                Manufacturer = RobotManufacturer.Unknown,
                ModelName = "serial_chain",
                Family = "serial",
                AxisCount = tip.Chain.Joints.Length,
                JointLimits = limits,
                BaseFrame = BaseFrame.Identity,
                ToolFrame = toolFrame,
                SourceNote = "Motus Serial Chain",
            };

            var model = new RobotModel(preset, BuildCapsuleCollision(tip.Chain, lengths, rail));
            var goo = new RobotModelGoo(model)
            {
                Chain = tip.Chain,
                Tree = tree,
                PreviewGeometry = model.CollisionModel,
            };

            if (basePl.IsValid)
                goo.BaseFrameOverride = FrameConversion.FromPlane(basePl);

            if (home.Count > 0)
            {
                var q = new double[preset.AxisCount];
                for (var i = 0; i < q.Length; i++)
                    q[i] = i < home.Count ? home[i] : 0;
                goo.PreviewHome = new JointState(q);
            }

            ApplyPreview(goo, sourcePath: $"serial:{tree.Fingerprint}");
            da.SetData(0, goo);
        }
        catch (Exception ex)
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    private static RobotCollisionModel BuildCapsuleCollision(
        SerialJointChain chain,
        IReadOnlyList<double> lengths,
        bool rail)
    {
        var links = new List<LinkCollisionGeometry>(chain.Joints.Length);
        for (var i = 0; i < chain.Joints.Length; i++)
        {
            var len = i < lengths.Count ? Math.Abs(lengths[i]) : 0.2;
            if (rail && i == 0)
                len = Math.Max(len * 0.5, 0.05); // visual half-stroke placeholder
            var half = Math.Max(len * 0.5, 0.02);
            var radius = 0.04;
            // Capsule along local +Z; offset so it spans the link roughly
            var pose = new Frame(0, 0, half, 1, 0, 0, 0);
            if (chain.Joints[i].Motion == JointMotionType.Prismatic)
                pose = new Frame(0, 0, half, 1, 0, 0, 0);
            else if (i > 0 || !rail)
                pose = new Frame(half, 0, 0, 0, 0, 0, 1); // along +X for revolute segments

            var name = i == chain.Joints.Length - 1 ? "tool0" : $"link{i + 1}";
            links.Add(new LinkCollisionGeometry(
                i,
                name,
                CollisionObject.Capsule(name, pose, radius, half)));
        }

        return new RobotCollisionModel(links);
    }

    public override Guid ComponentGuid => new("c8f2a1d0-4e3b-4a7c-9d1e-2b6f8a0c5e71");
}
