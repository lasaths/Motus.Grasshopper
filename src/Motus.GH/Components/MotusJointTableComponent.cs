using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>
/// Wave 2: one Joint Table → Motus.NET tree (branching OK). Not Link×N spaghetti.
/// Rows: Parent, Child, Type (R/P/C/F), Ox, Oy, Oz [, Ax, Ay, Az, Lo, Hi] — parallel lists.
/// </summary>
public sealed class MotusJointTableComponent : RobotSourceComponentBase
{
    public MotusJointTableComponent()
        : base(
            "Motus Joint Table",
            "JointTbl",
            "Build a Motus robot from a joint table (parent/child/type/origin). Same Robot goo as Motus Robot.",
            "tree-structure")
    {
    }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Parent", "Par", "Parent link names (first row root, usually base_link)", GH_ParamAccess.list);
        p.AddTextParameter("Child", "Ch", "Child link names", GH_ParamAccess.list);
        p.AddTextParameter("Type", "Ty", "R / P / C / F per joint", GH_ParamAccess.list);
        p.AddNumberParameter("Ox", "Ox", "Joint origin X (m)", GH_ParamAccess.list);
        p.AddNumberParameter("Oy", "Oy", "Joint origin Y (m)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Oz", "Oz", "Joint origin Z (m)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("Name", "N", "Optional joint names", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Base", "B", "Optional base frame (or use Mobility SE2)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Home", "Q", "Optional home driver q", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Mobility", "Mob", "Optional holonomic SE2: X, Y, Yaw(rad) — overrides Base origin/yaw", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Robot model (same as Motus Robot)", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var parents = new List<string>();
        var children = new List<string>();
        var types = new List<string>();
        var ox = new List<double>();
        var oy = new List<double>();
        var oz = new List<double>();
        var names = new List<string>();
        if (!da.GetDataList(0, parents) || parents.Count == 0
            || !da.GetDataList(1, children)
            || !da.GetDataList(2, types)
            || !da.GetDataList(3, ox))
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Wire Parent, Child, Type, Ox lists (same length).");
            return;
        }

        da.GetDataList(4, oy);
        da.GetDataList(5, oz);
        da.GetDataList(6, names);
        var basePl = Plane.Unset;
        da.GetData(7, ref basePl);
        var home = new List<double>();
        da.GetDataList(8, home);
        var mob = new List<double>();
        da.GetDataList(9, mob);

        var n = parents.Count;
        if (children.Count != n || types.Count != n || ox.Count != n)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Parent/Child/Type/Ox must be the same length.");
            return;
        }

        try
        {
            var rows = new JointTableRow[n];
            for (var i = 0; i < n; i++)
            {
                rows[i] = new JointTableRow(
                    i < names.Count && !string.IsNullOrWhiteSpace(names[i]) ? names[i] : $"j{i}",
                    parents[i],
                    children[i],
                    types[i],
                    ox[i],
                    i < oy.Count ? oy[i] : 0,
                    i < oz.Count ? oz[i] : 0,
                    0, 0, 1,
                    -Math.PI, Math.PI);
            }

            var tree = JointTableTrees.FromRows(rows, "joint_table");
            var tipLink = children[^1];
            var tip = tree.ExtractSerialTip(parents[0], tipLink);
            var limits = new List<JointLimit>(tree.DriverCount);
            for (var i = 0; i < tree.DriverCount; i++)
            {
                var j = tree.Joints[tree.DriverJointIndices[i]];
                var vel = j.Velocity ?? Math.PI;
                limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
            }

            var preset = new RobotPreset
            {
                Manufacturer = RobotManufacturer.Unknown,
                ModelName = "joint_table",
                Family = "serial",
                AxisCount = tip.Chain.Joints.Length,
                JointLimits = limits,
                BaseFrame = BaseFrame.Identity,
                ToolFrame = tip.TipToolOffset is { } off
                    ? new ToolFrame(off, "tool")
                    : ToolFrame.Identity,
                SourceNote = "Motus Joint Table",
            };

            var model = new RobotModel(preset);
            var goo = new RobotModelGoo(model)
            {
                Chain = tip.Chain,
                Tree = tree,
            };

            if (mob.Count >= 3)
                goo.BaseFrameOverride = new MobilityModel.HolonomicSE2(mob[0], mob[1], mob[2]).BaseFrame;
            else if (basePl.IsValid)
                goo.BaseFrameOverride = FrameConversion.FromPlane(basePl);

            if (home.Count > 0)
            {
                var q = new double[preset.AxisCount];
                for (var i = 0; i < q.Length; i++)
                    q[i] = i < home.Count ? home[i] : 0;
                goo.PreviewHome = new JointState(q);
            }

            ApplyPreview(goo, sourcePath: $"jointtbl:{tree.Fingerprint}");
            da.SetData(0, goo);
        }
        catch (Exception ex)
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new("d9e3b2c1-5f4a-4b8d-9e2f-3c7a1d0b6f82");
}
