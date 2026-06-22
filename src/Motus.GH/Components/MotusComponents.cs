using Grasshopper.Kernel;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Resources;
using Motus.Presets;
using Motus.Rhino;
using Rhino.Geometry;

namespace Motus.GH.Components;

public abstract class MotusComponentBase : GH_Component
{
    protected MotusComponentBase(string name, string nickname, string desc, string sub)
        : base(name, nickname, desc, "Motus", sub) { }

    protected override System.Drawing.Bitmap Icon => MotusIcon.Get();
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
    public MotusTrajectoryToPlanesComponent() : base("Motus Trajectory to Planes", "ToPl", "TCP planes via FK", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
        p.AddGenericParameter("Base", "B", "Base frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Tool", "Tf", "Tool frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddPlaneParameter("Planes", "P", "TCP planes", GH_ParamAccess.list);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectory(da, 0, out var t)) return;
        var baseF = GhExtract.OptionalBaseFrame(da, 1);
        var tool = GhExtract.OptionalToolFrame(da, 2);
        var planes = t.Points.Select(pt => KinematicsPreview.TcpPlane(t.Robot, pt.JointState, baseF, tool)).ToList();
        da.SetDataList(0, planes);
    }
    public override Guid ComponentGuid => new Guid("2957489a-d4bd-429d-8de3-6b5390640851");
}

public sealed class MotusTrajectoryToPosesComponent : MotusComponentBase
{
    public MotusTrajectoryToPosesComponent() : base("Motus Trajectory to Poses", "ToPs", "TCP frames via FK", "Export") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
        p.AddGenericParameter("Base", "B", "Base frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Tool", "Tf", "Tool frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Frames", "F", "Frames per point", GH_ParamAccess.list);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectory(da, 0, out var t)) return;
        var baseF = GhExtract.OptionalBaseFrame(da, 1);
        var tool = GhExtract.OptionalToolFrame(da, 2);
        var fk = KinematicsPreview.TryFk(t.Robot);
        var b = KinematicsPreview.ResolveBase(t.Robot, baseF);
        var tf = KinematicsPreview.ResolveTool(t.Robot, tool);
        var frames = t.Points.Select(pt =>
        {
            if (fk is null) return new FrameGoo(b.Frame);
            return new FrameGoo(fk.ComputeTcp(pt.JointState, b, tf).Tcp);
        }).ToList();
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
    public MotusPreviewRobotComponent() : base("Motus Preview Robot", "PrevRb", "FK link preview", "Preview") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("State", "S", "Joint state", GH_ParamAccess.item);
        p.AddGenericParameter("Base", "B", "Base frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Tool", "Tf", "Tool frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddLineParameter("Links", "L", "Link lines", GH_ParamAccess.list);
        p.AddMeshParameter("Meshes", "M", "Link meshes", GH_ParamAccess.list);
        p.AddPlaneParameter("Tool", "T", "TCP plane", GH_ParamAccess.item);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryRobot(da, 0, out var robot) || !GhExtract.TryJointState(da, 1, out var state)) return;
        var baseF = GhExtract.OptionalBaseFrame(da, 2);
        var tool = GhExtract.OptionalToolFrame(da, 3);
        da.SetDataList(0, KinematicsPreview.LinkLines(robot, state, baseF, tool).ToList());
        da.SetDataList(1, KinematicsPreview.LinkMeshes(robot, state, baseF, tool).ToList());
        da.SetData(2, KinematicsPreview.TcpPlane(robot, state, baseF, tool));
    }
    public override Guid ComponentGuid => new Guid("458ed2f4-5ce1-4541-8df4-bc4ff9fbee00");
}

public sealed class MotusPreviewTcpPathComponent : MotusComponentBase
{
    public MotusPreviewTcpPathComponent() : base("Motus Preview TCP Path", "PrevTCP", "FK TCP path", "Preview") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
        p.AddGenericParameter("Base", "B", "Base frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Tool", "Tf", "Tool frame override (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddCurveParameter("Path", "P", "TCP polyline", GH_ParamAccess.item);
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectory(da, 0, out var t)) return;
        var baseF = GhExtract.OptionalBaseFrame(da, 1);
        var tool = GhExtract.OptionalToolFrame(da, 2);
        var states = t.Points.Select(p => p.JointState);
        var pl = KinematicsPreview.TcpPath(t.Robot, states, baseF, tool);
        da.SetData(0, pl.Count >= 2 ? pl.ToNurbsCurve() : null);
    }
    public override Guid ComponentGuid => new Guid("7ba0b37a-7508-47da-8ac3-c2023d52270d");
}

public sealed class MotusPreviewTrajectoryComponent : MotusComponentBase
{
    public MotusPreviewTrajectoryComponent() : base("Motus Preview Trajectory", "PrevTr", "Start/goal + invalid segments", "Preview") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Trajectory", GH_ParamAccess.item);
        p.AddGenericParameter("Collision", "C", "Collision scene for segment check (optional)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("CheckAccel", "A", "Check acceleration", GH_ParamAccess.item, true);
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Start", "S", "Start TCP", GH_ParamAccess.item);
        p.AddPointParameter("Goal", "G", "Goal TCP", GH_ParamAccess.item);
        p.AddLineParameter("Valid", "V", "Valid TCP segments", GH_ParamAccess.list);
        p.AddLineParameter("Invalid", "I", "Invalid TCP segments", GH_ParamAccess.list);
        p.AddLineParameter("GoalLinks", "L", "Goal link lines", GH_ParamAccess.list);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectory(da, 0, out var t) || t.Points.Count == 0) return;
        var scene = GhExtract.OptionalCollisionScene(da, 1);
        var checkAccel = true;
        da.GetData(2, ref checkAccel);

        ICollisionChecker? checker = null;
        if (scene is not null && KinematicsProfiles.TryGet(t.Robot.Preset, out _))
            checker = new SphereCollisionChecker(t.Robot.Preset);

        var opts = new TrajectoryValidationOptions { CollisionChecker = checker, CollisionScene = scene, CheckAcceleration = checkAccel };
        KinematicsPreview.TrajectorySegments(t.Robot, t, opts, out var valid, out var invalid);

        var start = t.Points[0].JointState;
        var goal = t.Points[^1].JointState;
        var goalLines = KinematicsPreview.LinkLines(t.Robot, goal).ToList();

        da.SetData(0, KinematicsPreview.TcpPlane(t.Robot, start).Origin);
        da.SetData(1, KinematicsPreview.TcpPlane(t.Robot, goal).Origin);
        da.SetDataList(2, valid);
        da.SetDataList(3, invalid);
        da.SetDataList(4, goalLines);
    }
    public override Guid ComponentGuid => new Guid("45a84387-3f82-443a-a6f5-309a8c3be32c");
}
