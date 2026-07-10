using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using GH_IO.Serialization;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Resources;
using Motus.GH.UI;
using Motus.Presets;
using Motus.Rhino;
using Rhino;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;
using System.Globalization;

namespace Motus.GH.Components;

public abstract class MotusComponentBase : GH_Component
{
    private readonly string _iconName;

    protected MotusComponentBase(string name, string nickname, string desc, string sub, string iconName = "cube")
        : base(name, nickname, desc, "Motus", sub) => _iconName = iconName;

    protected override System.Drawing.Bitmap Icon => MotusIcon.Get(_iconName);
}

public sealed class MotusRobotComponent : MotusComponentBase
{
    public MotusRobotComponent() : base("Motus Robot", "Robot", "Pick a bundled robot preset (dropdown) or load one from JSON", "Model", "cube") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Model", "M", "Preset model name (UR5e, KR 6 R900, …)", GH_ParamAccess.item, "UR5e");
        p.AddTextParameter("JsonPath", "J", "Path to a preset JSON (overrides Model when set)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Base", "B", "Optional base frame override (TCP goals are in this frame)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Tool", "T", "Optional tool TCP frame override", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p) => p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
    public override void AddedToDocument(GH_Document doc)
    {
        base.AddedToDocument(doc);
        if (Params.Input[0].SourceCount > 0) return;
        // Defer document mutation: adding the value list and wiring it as a source while
        // GH is still placing this component blocks/hangs the canvas. Run it once the
        // current document operation has finished.
        doc.ScheduleSolution(1, _ => GhValueList.AttachDropdown(this, 0, PresetLoader.ListAvailableModels()));
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        var name = "UR5e";
        var jsonPath = "";
        var basePl = Plane.Unset;
        var toolPl = Plane.Unset;
        da.GetData(0, ref name);
        da.GetData(1, ref jsonPath);
        da.GetData(2, ref basePl);
        da.GetData(3, ref toolPl);
        try
        {
            var model = string.IsNullOrWhiteSpace(jsonPath)
                ? PresetLoader.LoadRobotModelByName(name)
                : PresetLoader.LoadRobotModelFromFile(jsonPath);
            var goo = new RobotModelGoo(model);
            if (basePl.IsValid) goo.BaseFrameOverride = FrameConversion.FromPlane(basePl);
            if (toolPl.IsValid) goo.ToolFrameOverride = FrameConversion.FromPlane(toolPl);
            da.SetData(0, goo);
        }
        catch (Exception ex) { AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message); }
    }
    public override Guid ComponentGuid => new Guid("aa3e8488-943e-426f-b205-e8db5f684998");
}

public sealed class MotusJointStateComponent : MotusComponentBase
{
    private bool _useDegrees;

    public MotusJointStateComponent() : base("Motus Joint State", "Joints", "Create joint state (radians)", "Model", "gear-six") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddAngleParameter("Joints", "J", "Joint angles (right-click J input to toggle °)", GH_ParamAccess.list);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("State", "S", "Joint state", GH_ParamAccess.item);

    protected override void BeforeSolveInstance()
    {
        _useDegrees = Params.Input[0] is Param_Number pn && pn.UseDegrees;
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var vals = new List<double>();
        if (!da.GetDataList(0, vals) || vals.Count == 0)
        {
            var text = "";
            if (da.GetData(0, ref text) && !string.IsNullOrWhiteSpace(text))
            {
                vals = text.Split(['\n', '\r', ',', ';'], StringSplitOptions.RemoveEmptyEntries)
                    .Select(s => double.Parse(s.Trim(), CultureInfo.InvariantCulture))
                    .ToList();
            }
        }
        if (vals.Count == 0) return;

        var arr = _useDegrees
            ? vals.Select(RhinoMath.ToRadians).ToArray()
            : vals.ToArray();
        da.SetData(0, new JointStateGoo(new JointState(arr)));
    }

    public override Guid ComponentGuid => new Guid("380f17c2-5d5f-4f77-a251-8309f25ef61e");
}

public sealed class MotusTcpPoseComponent : MotusComponentBase
{
    public MotusTcpPoseComponent()
        : base("Motus TCP Pose", "TCP", "Forward kinematics: joint state to TCP plane in base frame", "Model", "crosshair") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Robot", "Rb", "Robot model", GH_ParamAccess.item);
        p.AddGenericParameter("State", "S", "Joint state", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddPlaneParameter("Plane", "P", "TCP pose in robot base frame (position + orientation)", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryRobotGoo(da, 0, out var robotGoo)) return;
        JointStateGoo? stateGoo = null;
        if (!da.GetData(1, ref stateGoo) || stateGoo?.Value is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "State must be a joint state.");
            return;
        }

        var ctx = RobotContext.FromGoo(robotGoo);
        if (stateGoo.Value.AxisCount != ctx.Model.Preset.AxisCount)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                $"Expected {ctx.Model.Preset.AxisCount} joints, got {stateGoo.Value.AxisCount}.");
            return;
        }

        if (KinematicsPreview.TryFk(ctx.Model, ctx.Chain) is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "FK is not available for this robot model.");
            return;
        }

        var plane = KinematicsPreview.TcpPlane(ctx.Model, stateGoo.Value, ctx.Chain, ctx.Base, ctx.Tool);
        da.SetData(0, plane);
    }

    public override Guid ComponentGuid => new Guid("f1a2b3c4-d5e6-4789-a123-4567890abcde");
}

public sealed class MotusPreviewComponent : MotusComponentBase
{
    private static readonly Color CurrentColor = Color.FromArgb(200, 0, 196, 154);
    private static readonly Color StartColor = Color.FromArgb(90, 220, 220, 220);
    private static readonly Color PathColor = Color.FromArgb(180, 255, 255, 255);
    private static readonly Color InvalidColor = Color.FromArgb(220, 220, 60, 60);

    private Trajectory? _trajectory;
    private DateTime _playStartUtc;
    private bool _playing;
    private int _index;
    private bool _showStart;

    // Per-trajectory outputs (TCP path, invalid segments) don't change frame-to-frame,
    // so cache them and only recompute when the trajectory reference changes. Recomputing
    // them on every animation tick freezes the canvas for non-trivial trajectories.
    private Trajectory? _staticsFor;
    private global::Rhino.Geometry.Curve? _tcpCurve;
    private List<global::Rhino.Geometry.Line> _invalidSegments = new();
    private List<Mesh> _currentMeshes = new();
    private List<Mesh> _startMeshes = new();

    public MotusPreviewComponent() : base("Motus Preview", "Preview", "Animated FK preview; click Play/Stop on the component", "Preview", "eye") { }

    public override void CreateAttributes() =>
        m_attributes = new ButtonAttributes(this, () => _playing ? "\u25A0 Stop" : "\u25B6 Play", () => _playing, TogglePlayback);

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Motus trajectory from Motus Plan", GH_ParamAccess.item);
        p.AddBooleanParameter("ShowStart", "S", "Also preview the trajectory start pose as a ghost", GH_ParamAccess.item, false);
        p.AddGenericParameter("Robot", "Rb", "Optional robot/context override for external trajectories", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddMeshParameter("Meshes", "M", "Link meshes at the current frame", GH_ParamAccess.list);
        p.AddLineParameter("Links", "L", "Link lines at the current frame", GH_ParamAccess.list);
        p.AddCurveParameter("TCP Path", "P", "Full TCP polyline via FK", GH_ParamAccess.item);
        p.AddGenericParameter("State", "S", "Joint state at the current frame", GH_ParamAccess.item);
        p.AddNumberParameter("Time", "Tm", "Elapsed trajectory time at current frame (seconds)", GH_ParamAccess.item);
        p.AddIntegerParameter("Index", "I", "Current waypoint index (0-based)", GH_ParamAccess.item);
        p.AddLineParameter("Invalid", "X", "Invalid TCP segments (joint/velocity/acceleration limits)", GH_ParamAccess.list);
    }

    public override BoundingBox ClippingBox
    {
        get
        {
            var bb = BoundingBox.Empty;
            foreach (var mesh in _currentMeshes)
                bb.Union(mesh.GetBoundingBox(false));
            if (_showStart)
            {
                foreach (var mesh in _startMeshes)
                    bb.Union(mesh.GetBoundingBox(false));
            }
            if (_tcpCurve is not null)
                bb.Union(_tcpCurve.GetBoundingBox(false));
            return bb.IsValid ? bb : BoundingBox.Unset;
        }
    }

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (Locked) return;

        var currentMat = new DisplayMaterial(CurrentColor) { Transparency = 0.2 };
        foreach (var mesh in _currentMeshes)
            args.Display.DrawMeshShaded(mesh, currentMat);

        if (!_showStart) return;
        var startMat = new DisplayMaterial(StartColor) { Transparency = 0.55 };
        foreach (var mesh in _startMeshes)
            args.Display.DrawMeshShaded(mesh, startMat);
    }

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        if (Locked) return;

        if (_tcpCurve is not null)
            args.Display.DrawCurve(_tcpCurve, PathColor, 2);

        foreach (var line in _invalidSegments)
            args.Display.DrawLine(line, InvalidColor, 3);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("ShowStart", _showStart);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("ShowStart"))
            _showStart = reader.GetBoolean("ShowStart");
        return base.Read(reader);
    }

    public override void RemovedFromDocument(GH_Document doc)
    {
        _playing = false;
        base.RemovedFromDocument(doc);
    }

    private void TogglePlayback()
    {
        if (_playing) _playing = false;
        else if (_trajectory?.Points.Count > 0)
        {
            _playing = true;
            _playStartUtc = DateTime.UtcNow;
            _index = 0;
        }
        ExpireSolution(true);
    }

    private void ScheduleTick()
    {
        if (!_playing || OnPingDocument() is not GH_Document doc) return;
        doc.ScheduleSolution(33, _ => ExpireSolution(false));
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectoryGoo(da, 0, out var trajGoo)) return;
        var t = trajGoo.Value!;
        var ctx = trajGoo.Context();
        var previewGeometry = trajGoo.PreviewGeometry ?? ctx.Model.CollisionModel;
        var hasRobotOverride = GhExtract.TryRobotGoo(da, 2, out var robotOverride);
        if (hasRobotOverride)
        {
            ctx = RobotContext.FromGoo(robotOverride);
            previewGeometry = robotOverride.PreviewGeometry ?? ctx.Model.CollisionModel ?? previewGeometry;
        }
        _trajectory = t;
        if (t.Points.Count == 0) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Trajectory has no points."); return; }

        da.GetData(1, ref _showStart);

        if (t.Robot.Preset.AxisCount != ctx.Model.Preset.AxisCount)
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                $"Trajectory axis count ({t.Robot.Preset.AxisCount}) differs from preview robot axis count ({ctx.Model.Preset.AxisCount}).");
        }

        var previewPoints = BuildPreviewPoints(t, ctx.Model, out var jointRemapUsed);
        if (hasRobotOverride && !jointRemapUsed && !ReferenceEquals(t.Robot, ctx.Model))
        {
            AddRuntimeMessage(
                GH_RuntimeMessageLevel.Warning,
                "Robot override joint-name remap not available; preview uses source joint order. " +
                "Connect the trajectory's robot for exact geometry or align joint names.");
        }

        JointState state;
        double timeSeconds;
        if (_playing)
        {
            var elapsed = (DateTime.UtcNow - _playStartUtc).TotalSeconds;
            state = TrajectoryInterpolation.AtTime(new Trajectory(ctx.Model, previewPoints), elapsed, out _index);
            timeSeconds = elapsed;
            if (elapsed >= t.DurationSeconds) _playing = false;
            else ScheduleTick();
        }
        else
        {
            _index = Math.Clamp(_index, 0, previewPoints.Count - 1);
            state = previewPoints[_index].JointState;
            timeSeconds = previewPoints[_index].TimeSeconds;
        }

        if (!ReferenceEquals(_staticsFor, t))
        {
            var pl = KinematicsPreview.TcpPath(ctx.Model, previewPoints.Select(p => p.JointState), ctx.Chain, ctx.Base, ctx.Tool);
            _tcpCurve = pl.Count >= 2 ? pl.ToNurbsCurve() : null;
            KinematicsPreview.TrajectorySegments(
                ctx.Model,
                new Trajectory(ctx.Model, previewPoints),
                new TrajectoryValidationOptions(),
                out _,
                out var invalid,
                ctx.Chain,
                ctx.Base,
                ctx.Tool);
            _invalidSegments = invalid;
            _staticsFor = t;
        }

        _currentMeshes = KinematicsPreview.LinkMeshes(ctx.Model, state, previewGeometry, ctx.Chain, ctx.Base, ctx.Tool).ToList();
        _startMeshes = _showStart
            ? KinematicsPreview.LinkMeshes(ctx.Model, previewPoints[0].JointState, previewGeometry, ctx.Chain, ctx.Base, ctx.Tool).ToList()
            : new List<Mesh>();

        da.SetDataList(0, _currentMeshes);
        da.SetDataList(1, KinematicsPreview.LinkLines(ctx.Model, state, ctx.Chain, ctx.Base, ctx.Tool).ToList());
        da.SetData(2, _tcpCurve);
        da.SetData(3, new JointStateGoo(state));
        da.SetData(4, timeSeconds);
        da.SetData(5, _index);
        da.SetDataList(6, _invalidSegments);
        ExpirePreview(true);
    }

    private static List<TrajectoryPoint> BuildPreviewPoints(
        Trajectory sourceTrajectory,
        RobotModel previewRobot,
        out bool remapApplied)
    {
        remapApplied = false;
        if (sourceTrajectory.Points.Count == 0) return [];
        var mappedStates = TryBuildJointRemap(sourceTrajectory.Robot, previewRobot, out var map)
            ? sourceTrajectory.Points.Select(p => new JointState(RemapPositions(p.JointState.Positions, map))).ToList()
            : sourceTrajectory.Points.Select(p => p.JointState).ToList();
        remapApplied = map.Length > 0;

        var points = new List<TrajectoryPoint>(sourceTrajectory.Points.Count);
        for (var i = 0; i < sourceTrajectory.Points.Count; i++)
            points.Add(new TrajectoryPoint(sourceTrajectory.Points[i].TimeSeconds, mappedStates[i]));
        return points;
    }

    private static bool TryBuildJointRemap(RobotModel sourceRobot, RobotModel targetRobot, out int[] map)
    {
        map = [];
        if (sourceRobot.Preset.AxisCount != targetRobot.Preset.AxisCount) return false;
        var sourceNames = sourceRobot.JointNames;
        var targetNames = targetRobot.JointNames;
        if (sourceNames is null || targetNames is null) return false;
        if (sourceNames.Count != sourceRobot.Preset.AxisCount || targetNames.Count != targetRobot.Preset.AxisCount) return false;

        var sourceIndex = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
        for (var i = 0; i < sourceNames.Count; i++)
            sourceIndex[sourceNames[i]] = i;

        map = new int[targetNames.Count];
        for (var i = 0; i < targetNames.Count; i++)
        {
            if (!sourceIndex.TryGetValue(targetNames[i], out var idx))
            {
                map = [];
                return false;
            }
            map[i] = idx;
        }
        return true;
    }

    private static double[] RemapPositions(IReadOnlyList<double> sourcePositions, IReadOnlyList<int> map)
    {
        var remapped = new double[map.Count];
        for (var i = 0; i < map.Count; i++)
            remapped[i] = sourcePositions[map[i]];
        return remapped;
    }

    public override Guid ComponentGuid => new Guid("d4a8f1c2-3e5b-4a7d-9c1e-8f2b6d4e0a91");
}

public sealed class MotusTrajectoryDataComponent : MotusComponentBase
{
    public MotusTrajectoryDataComponent() : base("Motus Trajectory Data", "Data", "TCP planes, waypoint times, and per-axis joint series", "Export", "grid-four") { }
    protected override void RegisterInputParams(GH_InputParamManager p) => p.AddGenericParameter("Trajectory", "T", "Motus trajectory from Motus Plan", GH_ParamAccess.item);
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPlaneParameter("Planes", "P", "TCP planes via FK", GH_ParamAccess.list);
        p.AddNumberParameter("Times", "Tm", "Elapsed time at each waypoint (seconds)", GH_ParamAccess.list);
        p.AddNumberParameter("Joints", "J", "Joint angles (rad); branch {i} = axis i, items = waypoints", GH_ParamAccess.tree);
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectoryGoo(da, 0, out var trajGoo)) return;
        var t = trajGoo.Value!;
        var ctx = trajGoo.Context();
        if (t.Points.Count == 0) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Trajectory has no points."); return; }

        var planes = t.Points.Select(pt => KinematicsPreview.TcpPlane(ctx.Model, pt.JointState, ctx.Chain, ctx.Base, ctx.Tool)).ToList();
        var times = t.Points.Select(p => p.TimeSeconds).ToList();
        var tree = new GH_Structure<GH_Number>();
        for (var j = 0; j < t.Robot.Preset.AxisCount; j++)
        {
            var path = new GH_Path(j);
            foreach (var pt in t.Points)
                tree.Append(new GH_Number(pt.JointState.Positions[j]), path);
        }
        da.SetDataList(0, planes);
        da.SetDataList(1, times);
        da.SetDataTree(2, tree);
    }
    public override Guid ComponentGuid => new Guid("a72b5cfa-5cf5-4e54-a5cd-943e2aae82da");
}

public sealed class MotusExportComponent : MotusComponentBase
{
    public MotusExportComponent() : base("Motus Export", "Export", "Serialize a trajectory to JSON and CSV", "Export", "export") { }
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddGenericParameter("Trajectory", "T", "Motus trajectory from Motus Plan", GH_ParamAccess.item);
        p.AddBooleanParameter("Retime", "R", "Apply bottleneck path retiming before export", GH_ParamAccess.item, true);
        p[p.ParamCount - 1].Optional = true;
        p.AddBooleanParameter("Validate", "V", "Validate limits/velocity after retiming", GH_ParamAccess.item, false);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddTextParameter("Json", "J", "Trajectory as JSON", GH_ParamAccess.item);
        p.AddTextParameter("Csv", "C", "Trajectory as CSV", GH_ParamAccess.item);
        p.AddTextParameter("Validation", "Val", "Validation summary when Validate=true", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectory(da, 0, out var t)) return;
        var retime = true;
        var validate = false;
        da.GetData(1, ref retime);
        da.GetData(2, ref validate);

        var result = TrajectoryExport.Export(t, new TrajectoryExportOptions { Retime = retime, Validate = validate });
        da.SetData(0, result.Json);
        da.SetData(1, result.Csv);
        if (validate && result.Validation is not null)
        {
            da.SetData(2, result.Validation.IsValid
                ? "Valid."
                : string.Join("; ", result.Validation.Errors));
        }
    }
    public override Guid ComponentGuid => new Guid("0a443b6f-605b-48e3-843c-cd0a709f8379");
}
