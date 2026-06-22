using Grasshopper.Kernel;
using Motus.Core;
using Motus.GH.Data;
using Motus.Presets;
using Motus.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

public abstract class MotusComponentBase : GH_Component
{
    protected MotusComponentBase(string name, string nickname, string desc, string sub)
        : base(name, nickname, desc, "Motus", sub) { }

    protected override System.Drawing.Bitmap Icon => null!;
}

public sealed class MotusRobotModelComponent : MotusComponentBase
{
    public MotusRobotModelComponent() : base("Motus Robot Model", "Robot", "Wrap a robot preset as a model", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Preset", "P", "Robot preset", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Model", "M", "Robot model", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        object? o = null;
        if (!da.GetData(0, ref o)) return;
        var preset = o is RobotPreset rp ? rp : (o as dynamic)?.Value as RobotPreset;
        if (preset == null) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Expected RobotPreset."); return; }
        da.SetData(0, new RobotModelGoo(preset.ToModel()));
    }
    public override Guid ComponentGuid => new Guid("aa3e8488-943e-426f-b205-e8db5f684998");
}

public sealed class MotusUrPresetComponent : MotusComponentBase
{
    public MotusUrPresetComponent() : base("Motus UR Preset", "UR", "Load a Universal Robots preset", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddTextParameter("Model", "M", "Model name e.g. UR5e", GH_ParamAccess.item, "UR5e");
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Preset", "P", "Robot preset", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "UR5e";
        if (!da.GetData(0, ref name)) return;
        try { da.SetData(0, PresetLoader.LoadByModelName(name)); }
        catch (Exception ex) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
    }
    public override Guid ComponentGuid => new Guid("fffae605-7c51-4c47-bfe6-20eb540594da");
}

public sealed class MotusKukaPresetComponent : MotusComponentBase
{
    public MotusKukaPresetComponent() : base("Motus KUKA Preset", "KUKA", "Load a KUKA preset", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddTextParameter("Model", "M", "Model name e.g. KR 6 R900", GH_ParamAccess.item, "KR 6 R900");
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Preset", "P", "Robot preset", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "KR 6 R900";
        if (!da.GetData(0, ref name)) return;
        try { da.SetData(0, PresetLoader.LoadByModelName(name)); }
        catch (Exception ex) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
    }
    public override Guid ComponentGuid => new Guid("fd6defff-52bc-4abd-81e0-352cc5332fb8");
}

public sealed class MotusCustomRobotComponent : MotusComponentBase
{
    public MotusCustomRobotComponent() : base("Motus Custom Robot", "Custom", "Build robot from JSON preset path", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddTextParameter("JsonPath", "J", "Path to preset JSON", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Preset", "P", "Robot preset", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        if (!da.GetData(0, ref path) || string.IsNullOrWhiteSpace(path)) return;
        try { da.SetData(0, PresetLoader.LoadFromFile(path)); }
        catch (Exception ex) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
    }
    public override Guid ComponentGuid => new Guid("90a11b5a-2ef1-41f7-b8e6-f60223c1572b");
}

public sealed class MotusJointStateComponent : MotusComponentBase
{
    public MotusJointStateComponent() : base("Motus Joint State", "Joints", "Create joint state (radians)", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Joints", "J", "Joint angles in radians", GH_ParamAccess.list);
        p.AddBooleanParameter("UseDegrees", "D", "Interpret input as degrees", GH_ParamAccess.item, false);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("State", "S", "Joint state", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var vals = new List<double>();
        var deg = false;
        if (!da.GetDataList(0, vals)) return;
        da.GetData(1, ref deg);
        var arr = deg ? Units.ToRadians(vals.ToArray()) : vals.ToArray();
        da.SetData(0, new JointStateGoo(new JointState(arr)));
    }
    public override Guid ComponentGuid => new Guid("380f17c2-5d5f-4f77-a251-8309f25ef61e");
}

public sealed class MotusToolFrameComponent : MotusComponentBase
{
    public MotusToolFrameComponent() : base("Motus Tool Frame", "Tool", "Define tool frame from plane", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddPlaneParameter("Plane", "P", "Tool plane (meters)", GH_ParamAccess.item, Plane.WorldXY);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Tool", "T", "Tool frame", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var pl = Plane.WorldXY;
        if (!da.GetData(0, ref pl)) return;
        da.SetData(0, new ToolFrameGoo(new ToolFrame(FrameConversion.FromPlane(pl))));
    }
    public override Guid ComponentGuid => new Guid("f8235119-89ff-4bc8-a6be-196401e81226");
}

public sealed class MotusBaseFrameComponent : MotusComponentBase
{
    public MotusBaseFrameComponent() : base("Motus Base Frame", "Base", "Define base frame from plane", "Model") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddPlaneParameter("Plane", "P", "Base plane (meters)", GH_ParamAccess.item, Plane.WorldXY);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Base", "B", "Base frame", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var pl = Plane.WorldXY;
        if (!da.GetData(0, ref pl)) return;
        da.SetData(0, new BaseFrameGoo(new BaseFrame(FrameConversion.FromPlane(pl))));
    }
    public override Guid ComponentGuid => new Guid("ea9aae72-c7ec-4422-ab24-0906e0f78a95");
}

public sealed class MotusPlanJointPathComponent : MotusComponentBase
{
    private PlanningResult? _cached;
    private string _cacheKey = "";

    public MotusPlanJointPathComponent() : base("Motus Plan Joint Path", "Plan", "Joint-space linear plan (Run gate)", "Plan") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddBooleanParameter("Run", "R", "Plan only when true", GH_ParamAccess.item, false);
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("Start", "S", "Start joint state", GH_ParamAccess.item);
        p.AddGenericParameter("Goal", "G", "Goal joint state", GH_ParamAccess.item);
        p.AddNumberParameter("MaxStep", "St", "Max joint step (rad)", GH_ParamAccess.item, 0.05);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Planned trajectory", GH_ParamAccess.item);
        p.AddTextParameter("Status", "St", "Status message", GH_ParamAccess.item);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var run = false;
        if (!da.GetData(0, ref run) || !run)
        {
            if (_cached?.Success == true) da.SetData(0, new TrajectoryGoo(_cached.Trajectory!));
            da.SetData(1, run ? "" : "Idle (set Run=true to plan).");
            return;
        }

        RobotModelGoo? robotGoo = null; JointStateGoo? startGoo = null; JointStateGoo? goalGoo = null;
        var maxStep = 0.05;
        if (!da.GetData(1, ref robotGoo) || !da.GetData(2, ref startGoo) || !da.GetData(3, ref goalGoo)) return;
        da.GetData(4, ref maxStep);

        var key = $"{robotGoo!.Value.DisplayName}|{string.Join(",", startGoo!.Value.Positions)}|{string.Join(",", goalGoo!.Value.Positions)}|{maxStep}";
        if (key == _cacheKey && _cached != null)
        {
            if (_cached.Success) da.SetData(0, new TrajectoryGoo(_cached.Trajectory!));
            da.SetData(1, _cached.Success ? "Success (cached)." : string.Join("; ", _cached.Errors));
            da.SetDataList(2, _cached.Warnings);
            return;
        }

        var req = new PlanningRequest(robotGoo.Value, startGoo.Value, goalGoo.Value, new PlanningOptions { MaxJointStepRadians = maxStep });
        _cached = new JointLinearPlanner().Plan(req);
        _cacheKey = key;

        if (_cached.Success) da.SetData(0, new TrajectoryGoo(_cached.Trajectory!));
        da.SetData(1, _cached.Success ? "Success." : string.Join("; ", _cached.Errors));
        da.SetDataList(2, _cached.Warnings);
    }
    public override Guid ComponentGuid => new Guid("8bb0bae3-527f-4e80-a8a4-c8a88b7276de");
}

public sealed class MotusValidateTrajectoryComponent : MotusComponentBase
{
    public MotusValidateTrajectoryComponent() : base("Motus Validate Trajectory", "Valid", "Validate trajectory", "Plan") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddBooleanParameter("Valid", "V", "Is valid", GH_ParamAccess.item);
        p.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
        p.AddTextParameter("Warnings", "W", "Warnings", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        var r = new TrajectoryValidator().Validate(t!.Value);
        da.SetData(0, r.IsValid);
        da.SetDataList(1, r.Errors);
        da.SetDataList(2, r.Warnings);
    }
    public override Guid ComponentGuid => new Guid("81caa6c6-166e-4e58-8325-8c6df7270ce0");
}

public sealed class MotusTrajectoryInfoComponent : MotusComponentBase
{
    public MotusTrajectoryInfoComponent() : base("Motus Trajectory Info", "Info", "Trajectory summary", "Plan") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddIntegerParameter("Points", "P", "Point count", GH_ParamAccess.item);
        p.AddNumberParameter("Duration", "D", "Duration (s)", GH_ParamAccess.item);
        p.AddTextParameter("Robot", "R", "Robot name", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        da.SetData(0, t!.Value.Points.Count);
        da.SetData(1, t.Value.DurationSeconds);
        da.SetData(2, t.Value.Robot.DisplayName);
    }
    public override Guid ComponentGuid => new Guid("c195135d-2f94-44f5-9fab-fab3b55aabfd");
}

public sealed class MotusTrajectoryToJointListsComponent : MotusComponentBase
{
    public MotusTrajectoryToJointListsComponent() : base("Motus Trajectory to Joint Lists", "ToJ", "Export joint lists per axis", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddNumberParameter("Times", "T", "Time per point (s)", GH_ParamAccess.list);
        p.AddNumberParameter("Joints", "J", "Flat joint values (rad)", GH_ParamAccess.tree);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        var times = t!.Value.Points.Select(p => p.TimeSeconds).ToList();
        var tree = new Grasshopper.Kernel.Data.GH_Structure<Grasshopper.Kernel.Types.GH_Number>();
        for (var j = 0; j < t.Value.Robot.Preset.AxisCount; j++)
        {
            var path = new Grasshopper.Kernel.Data.GH_Path(j);
            foreach (var pt in t.Value.Points)
                tree.Append(new Grasshopper.Kernel.Types.GH_Number(pt.JointState.Positions[j]), path);
        }
        da.SetDataList(0, times);
        da.SetData(1, tree);
    }
    public override Guid ComponentGuid => new Guid("a72b5cfa-5cf5-4e54-a5cd-943e2aae82da");
}

public sealed class MotusTrajectoryToPlanesComponent : MotusComponentBase
{
    public MotusTrajectoryToPlanesComponent() : base("Motus Trajectory to Planes", "ToPl", "Placeholder TCP planes (base frame)", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddPlaneParameter("Planes", "P", "TCP planes (simplified)", GH_ParamAccess.list);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        var planes = t!.Value.Points.Select(pt => FrameConversion.ToPlane(t.Value.Robot.Preset.BaseFrame.Frame)).ToList();
        da.SetDataList(0, planes);
    }
    public override Guid ComponentGuid => new Guid("2957489a-d4bd-429d-8de3-6b5390640851");
}

public sealed class MotusTrajectoryToPosesComponent : MotusComponentBase
{
    public MotusTrajectoryToPosesComponent() : base("Motus Trajectory to Poses", "ToPs", "Export Motus frames", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Frames", "F", "Frames per point", GH_ParamAccess.list);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        var frames = t!.Value.Points.Select(_ => new FrameGoo(t.Value.Robot.Preset.BaseFrame.Frame)).ToList();
        da.SetDataList(0, frames);
    }
    public override Guid ComponentGuid => new Guid("bba81a6e-b7b8-498e-bfb4-25662c074a45");
}

public sealed class MotusTrajectoryToJsonComponent : MotusComponentBase
{
    public MotusTrajectoryToJsonComponent() : base("Motus Trajectory to JSON", "ToJson", "Export trajectory JSON", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddTextParameter("Json", "J", "JSON", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        da.SetData(0, TrajectoryExport.ToJson(t!.Value));
    }
    public override Guid ComponentGuid => new Guid("0a443b6f-605b-48e3-843c-cd0a709f8379");
}

public sealed class MotusTrajectoryToCsvComponent : MotusComponentBase
{
    public MotusTrajectoryToCsvComponent() : base("Motus Trajectory to CSV", "ToCsv", "Export trajectory CSV", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddTextParameter("Csv", "C", "CSV", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        da.SetData(0, TrajectoryExport.ToCsv(t!.Value));
    }
    public override Guid ComponentGuid => new Guid("955a1c3b-1108-4ecd-8111-4360ba3a202f");
}

public sealed class MotusPreviewRobotComponent : MotusComponentBase
{
    public MotusPreviewRobotComponent() : base("Motus Preview Robot", "PrevRb", "Preview stick robot", "Preview") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("State", "S", "Joint state", GH_ParamAccess.item);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddLineParameter("Links", "L", "Link lines", GH_ParamAccess.list);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        JointStateGoo? s = null;
        if (!da.GetData(1, ref s)) return;
        da.SetDataList(0, RobotPreview.StickLinks(s!.Value).ToList());
    }
    public override Guid ComponentGuid => new Guid("458ed2f4-5ce1-4541-8df4-bc4ff9fbee00");
}

public sealed class MotusPreviewTcpPathComponent : MotusComponentBase
{
    public MotusPreviewTcpPathComponent() : base("Motus Preview TCP Path", "PrevTCP", "Preview simplified TCP path", "Preview") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddCurveParameter("Path", "P", "TCP polyline", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t)) return;
        var states = t!.Value.Points.Select(p => p.JointState);
        da.SetData(0, RobotPreview.TcpPathFromJointStates(states).ToNurbsCurve());
    }
    public override Guid ComponentGuid => new Guid("7ba0b37a-7508-47da-8ac3-c2023d52270d");
}

public sealed class MotusPreviewTrajectoryComponent : MotusComponentBase
{
    public MotusPreviewTrajectoryComponent() : base("Motus Preview Trajectory", "PrevTr", "Preview start/goal markers", "Preview") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Start", "S", "Start point", GH_ParamAccess.item);
        p.AddPointParameter("Goal", "G", "Goal point", GH_ParamAccess.item);
        p.AddLineParameter("Links", "L", "Goal stick figure", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        TrajectoryGoo? t = null;
        if (!da.GetData(0, ref t) || t!.Value.Points.Count == 0) return;
        var start = t.Value.Points[0].JointState;
        var goal = t.Value.Points[^1].JointState;
        var startLines = RobotPreview.StickLinks(start).ToList();
        var goalPt = startLines.Count > 0 ? startLines[^1].To : Point3d.Origin;
        var startPt = startLines.Count > 0 ? startLines[0].From : Point3d.Origin;
        da.SetData(0, startPt);
        var goalLines = RobotPreview.StickLinks(goal).ToList();
        da.SetData(1, goalLines.Count > 0 ? goalLines[^1].To : goalPt);
        da.SetDataList(2, RobotPreview.StickLinks(goal).ToList());
    }
    public override Guid ComponentGuid => new Guid("45a84387-3f82-443a-a6f5-309a8c3be32c");
}
