using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH.Data;
using Motus.GH.Params;
using Motus.GH.Preview;
using Motus.GH.Rhino;
using Motus.GH.Urdf;
using Rhino.Geometry;

namespace Motus.GH.Components;

public sealed class MotusDescriptionRobotComponent : RobotSourceComponentBase
{
    public MotusDescriptionRobotComponent() : base("Motus Robot From Description", "FromDesc",
        "Build a plannable robot directly from URDF Assemble/Attach; no file export required", "tree-structure") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddParameter(new Param_MotusRobotDescription(), "Description", "D", "Authored robot description", GH_ParamAccess.item);
        p.AddTextParameter("Tip", "Tip", "Tip link; omit to use the description's selected tip", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("BaseLink", "B", "Root of the planning chain; omit for description root", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Base", "Bf", "Placement in world coordinates; XYZ axes are preserved", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("AllDrivers", "All", "Include side branches after tip joints; tip-descendant tool joints stay separate", GH_ParamAccess.item, false);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddParameter(new Param_MotusRobot(), "Robot", "Rb", "Robot → Plan / Program / Robot Info", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        RobotDescriptionGoo? description = null;
        if (!da.GetData(0, ref description) || description?.Value is null) { ClearPreview(); return; }
        string? tipName = null, baseName = null;
        var placement = Plane.Unset;
        var all = false;
        da.GetData(1, ref tipName); da.GetData(2, ref baseName); da.GetData(3, ref placement); da.GetData(4, ref all);
        try
        {
            var (tree, tip) = RobotDescriptionSession.Project(description.Value, baseName, tipName);
            if (tip is null) throw new ArgumentException("Choose a Tip link on Assemble or Robot From Description.");
            var resolvedTip = string.IsNullOrWhiteSpace(tipName) ? description.Value.TipLink! : tipName;
            var layout = PlanDofComposer.TipThenSideBranches(tree, tip.JointNames, resolvedTip);
            var count = all ? layout.JointNames.Count : tip.JointNames.Count;
            var names = layout.JointNames.Take(count).ToArray();
            var preset = new RobotPreset
            {
                Manufacturer = RobotManufacturer.Unknown, ModelName = description.Value.Name, Family = "serial",
                AxisCount = count, JointLimits = layout.Limits.Take(count).ToArray(),
                ToolFrame = tip.TipToolOffset is { } offset ? new ToolFrame(offset, resolvedTip) : ToolFrame.Identity
            };
            var model = new RobotModel(preset, MechanismPreviewGeometry.Build(description.Value, collision: true), names);
            var goo = new RobotModelGoo(model)
            {
                Chain = tip.Chain, Tree = tree, PreviewGeometry = MechanismPreviewGeometry.Build(description.Value),
                BaseFrameOverride = placement.IsValid ? FrameConversion.FromPlanePlate(placement) : null,
                TreeDriverHome = new JointState(description.Value.HomeQ?.ToArray() ?? new double[tree.DriverCount])
            };
            var homeByName = tree.DriverJointIndices.Select((ji, i) => (tree.Joints[ji].Name, Value: goo.TreeDriverHome.Positions[i]))
                .ToDictionary(v => v.Name, v => v.Value);
            goo.PreviewHome = new JointState(names.Select(n => homeByName[n]).ToArray());
            ApplyPreview(goo, sourcePath: $"description:{description.Value.Fingerprint}");
            da.SetData(0, goo);
        }
        catch (Exception ex) { ClearPreview(); AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
    }

    public override Guid ComponentGuid => new("5ecb9136-f695-46d7-9d36-5ee05628761e");
}

public sealed class MotusRobotInfoComponent : MotusComponentBase
{
    public MotusRobotInfoComponent() : base("Motus Robot Info", "RobotInfo",
        "Inspect planning joint order, per-axis units and collision body names for Touch", "Model", "list-plus") { }
    protected override void RegisterInputParams(GH_InputParamManager p) =>
        p.AddParameter(new Param_MotusRobot(), "Robot", "Rb", "Robot to inspect", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddTextParameter("Joints", "J", "Planning joint names in coordinate order", GH_ParamAccess.list);
        p.AddTextParameter("Units", "U", "Per-axis coordinate units: rad or m", GH_ParamAccess.list);
        p.AddTextParameter("Collision Bodies", "Bodies", "Collision body names; choose gripper bodies for Pick Place Touch", GH_ParamAccess.list);
        p.AddTextParameter("Family", "F", "Robot family", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryRobotGoo(da, 0, out var goo)) return;
        var robot = goo.EffectiveModel();
        da.SetDataList(0, robot.JointNames ?? Enumerable.Range(1, robot.Preset.AxisCount).Select(i => $"joint_{i}").ToArray());
        da.SetDataList(1, robot.Preset.JointLimits.Select(l => l.UnitLabel));
        var bodies = robot.CollisionModel?.Links.Select(l => l.LocalGeometry.Name) ?? [];
        if (robot.CollisionModel?.ToolGeometry is { } tool) bodies = bodies.Append(tool.Name);
        da.SetDataList(2, bodies.Distinct(StringComparer.OrdinalIgnoreCase));
        da.SetData(3, robot.Preset.Family);
    }
    public override Guid ComponentGuid => new("99b5f4ba-b62b-40d4-9ff2-89398b728af8");
}
