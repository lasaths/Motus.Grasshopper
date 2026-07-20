using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

/// <summary>
/// Wave 2: one Joint Table → Motus.NET tree (branching OK). Not Link×N spaghetti.
/// Plan/Joint State use the <b>tip path</b> only; full tree stays on goo for TreeFK preview.
/// </summary>
public sealed class MotusJointTableComponent : RobotSourceComponentBase
{
    public MotusJointTableComponent()
        : base(
            "Motus Joint Table",
            "JointTbl",
            "Build a Motus robot from a joint table (parent/child/type/origin). Plan uses tip path; side branches are TreeFK preview only.",
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
        p.AddTextParameter("Tip", "Tip", "Tip link for Plan/serial chain (default: last Child)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Base", "B", "Optional base frame (ignored when BaseSE2 wired)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddNumberParameter("Home", "Q", "Optional home q along tip path (Plan/Joint State order)", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
        // ponytail: SE2 pose only — not mobile RRT
        p.AddNumberParameter("BaseSE2", "SE2", "Optional base pose X, Y, Yaw(rad) — frame override only, not mobile planning", GH_ParamAccess.list);
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
        var tipLink = "";
        da.GetData(7, ref tipLink);
        var basePl = Plane.Unset;
        da.GetData(8, ref basePl);
        var home = new List<double>();
        da.GetDataList(9, home);
        var baseSe2 = new List<double>();
        da.GetDataList(10, baseSe2);

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
            var root = string.IsNullOrWhiteSpace(parents[0]) ? "base_link" : parents[0].Trim();
            if (string.IsNullOrWhiteSpace(tipLink))
                tipLink = children[^1];
            tipLink = tipLink.Trim();

            var tip = tree.ExtractSerialTip(root, tipLink);
            // Plan contract: AxisCount + limits must match tip-path actuated joints only.
            var limits = LimitsAlongTip(tree, tip.JointNames);
            if (limits.Count != tip.Chain.Joints.Length)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                    $"Tip path joint count mismatch ({limits.Count} vs {tip.Chain.Joints.Length}).");
                return;
            }

            if (tree.DriverCount != tip.Chain.Joints.Length)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    $"Tree has {tree.DriverCount} drivers; Plan/Joint State use tip path '{tipLink}' ({tip.Chain.Joints.Length} axes). Side branches are TreeFK preview only.");
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

            if (baseSe2.Count >= 3)
                goo.BaseFrameOverride = new MobilityModel.HolonomicSE2(baseSe2[0], baseSe2[1], baseSe2[2]).BaseFrame;
            else if (basePl.IsValid)
                goo.BaseFrameOverride = FrameConversion.FromPlane(basePl);

            if (home.Count > 0)
            {
                var q = new double[preset.AxisCount];
                for (var i = 0; i < q.Length; i++)
                    q[i] = i < home.Count ? home[i] : 0;
                var homeState = new JointState(q);
                var val = homeState.Validate(preset.JointLimits);
                if (!val.IsValid)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                        string.Join(" ", val.Errors));
                    return;
                }
                goo.PreviewHome = homeState;
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

    /// <summary>Joint limits in tip-path order (same as <see cref="SerialTipExtraction.JointNames"/>).</summary>
    private static List<JointLimit> LimitsAlongTip(KinematicTree tree, IReadOnlyList<string> tipJointNames)
    {
        var byName = new Dictionary<string, KinematicJoint>(tree.Joints.Count, StringComparer.OrdinalIgnoreCase);
        foreach (var j in tree.Joints)
            byName[j.Name] = j;

        var limits = new List<JointLimit>(tipJointNames.Count);
        foreach (var name in tipJointNames)
        {
            if (!byName.TryGetValue(name, out var j))
                throw new InvalidOperationException($"Tip joint '{name}' missing from tree.");
            var vel = j.Velocity ?? Math.PI;
            limits.Add(new JointLimit(j.Lower, j.Upper, vel, vel * 2));
        }
        return limits;
    }

    public override Guid ComponentGuid => new("d9e3b2c1-5f4a-4b8d-9e2f-3c7a1d0b6f82");
}
