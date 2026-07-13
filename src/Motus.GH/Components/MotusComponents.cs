using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;
using GH_IO.Serialization;
using Motus.Core;
using Motus.Geometry;
using Motus.GH;
using Motus.GH.Data;
using Motus.GH.Loaders;
using Motus.GH.Preview;
using Motus.GH.Resources;
using Motus.GH.Rhino;
using Motus.GH.UI;
using Motus.GH.Urdf;
using Rhino;
using Rhino.Display;
using Rhino.Geometry;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;

namespace Motus.GH.Components;

public abstract class MotusComponentBase : GH_Component
{
    private readonly string _iconName;
    private readonly string _subcategory;

    protected MotusComponentBase(string name, string nickname, string desc, string sub, string iconName = "cube")
        : base(name, nickname, desc, "Motus", sub)
    {
        _subcategory = sub;
        _iconName = iconName;
    }

    protected override System.Drawing.Bitmap Icon =>
        MotusIcon.Get(_iconName, MotusIcon.SubcategoryColor(_subcategory));
}

public abstract class RobotSourceComponentBase : MotusComponentBase
{
    protected List<Mesh> _previewMeshes = [];
    protected List<Mesh> _collisionPreviewMeshes = [];
    protected List<Line> _previewWires = [];
    private string? _previewKey;
    private bool _showCollisionPreview;

    protected RobotSourceComponentBase(string name, string nickname, string desc, string iconName = "cube")
        : base(name, nickname, desc, "Model", iconName) { }

    protected void ApplyPreview(RobotModelGoo goo, string? sourcePath)
    {
        var key = PreviewKey(goo, sourcePath, _showCollisionPreview);
        if (key == _previewKey && _previewMeshes.Count > 0)
            return;

        _previewKey = key;
        RobotViewportPreview.Build(goo, sourcePath, out _previewMeshes, out _previewWires);
        _collisionPreviewMeshes = _showCollisionPreview
            ? RobotViewportPreview.BuildPlanningCollisionMeshes(goo, sourcePath)
            : [];
        ExpirePreview(true);
    }

    protected void ClearPreview()
    {
        _previewKey = null;
        _previewMeshes = [];
        _collisionPreviewMeshes = [];
        _previewWires = [];
        ExpirePreview(true);
    }

    private static string PreviewKey(RobotModelGoo goo, string? sourcePath, bool showCollision) =>
        $"{sourcePath}|{goo.UrdfSourcePath}|{goo.Tool?.Name}|{goo.BaseFrameOverride}|{goo.PreviewGeometry?.Links.Count}|{goo.Chain?.Joints.Length}|col:{showCollision}";

    public override BoundingBox ClippingBox =>
        RobotViewportPreview.ComputeBounds(
            _collisionPreviewMeshes.Count > 0 ? _collisionPreviewMeshes : _previewMeshes,
            _previewWires);

    public override void DrawViewportMeshes(IGH_PreviewArgs args)
    {
        if (Locked) return;
        RobotViewportPreview.DrawMeshes(args, _previewMeshes);
        if (_showCollisionPreview)
            RobotViewportPreview.DrawCollisionMeshes(args, _collisionPreviewMeshes);
    }

    public override void DrawViewportWires(IGH_PreviewArgs args)
    {
        if (Locked) return;
        RobotViewportPreview.DrawWires(args, _previewWires, _previewMeshes.Count == 0);
    }

    public override void AppendAdditionalMenuItems(ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Preview collision meshes", CollisionPreviewMenuClick, true, _showCollisionPreview);
        base.AppendAdditionalMenuItems(menu);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("ShowCollisionPreview", _showCollisionPreview);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        if (reader.ItemExists("ShowCollisionPreview"))
            _showCollisionPreview = reader.GetBoolean("ShowCollisionPreview");
        return base.Read(reader);
    }

    private void CollisionPreviewMenuClick(object? sender, EventArgs e)
    {
        RecordUndoEvent("Preview collision meshes");
        _showCollisionPreview = !_showCollisionPreview;
        _previewKey = null;
        ExpireSolution(true);
    }
}

public sealed class MotusRobotComponent : RobotSourceComponentBase
{
    public MotusRobotComponent()
        : base("Motus Robot", "Robot", "Load a robot from URDF or .xacro; optional Base and Tool overrides", "file") { }

    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddTextParameter("Path", "P", "Path to .urdf or .xacro file", GH_ParamAccess.item);
        p.AddTextParameter("BaseLink", "B", "Base link name", GH_ParamAccess.item, "base_link");
        p[p.ParamCount - 1].Optional = true;
        p.AddTextParameter("TipLink", "T", "Tip link name", GH_ParamAccess.item, "tool0");
        p[p.ParamCount - 1].Optional = true;
        p.AddPlaneParameter("Base", "Bf", "Optional base frame override (TCP goals are in this frame)", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
        p.AddGenericParameter("Tool", "Tl", "Optional Motus Tool definition", GH_ParamAccess.item);
        p[p.ParamCount - 1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Robot model with URDF kinematics chain", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var path = "";
        var baseLink = "base_link";
        var tipLink = "tool0";
        var basePl = Plane.Unset;
        da.GetData(1, ref baseLink);
        da.GetData(2, ref tipLink);
        da.GetData(3, ref basePl);
        ToolGoo? toolGoo = null;
        da.GetData(4, ref toolGoo);

        if (!da.GetData(0, ref path) || string.IsNullOrWhiteSpace(path))
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "Set Path to a .urdf or .xacro file.");
            return;
        }

        try
        {
            var goo = UrdfRobotLoad.Load(path, baseLink, tipLink);
            if (basePl.IsValid) goo.BaseFrameOverride = FrameConversion.FromPlane(basePl);
            if (toolGoo?.Value is not null) goo.Tool = toolGoo.Value;
            ApplyPreview(goo, path);
            da.SetData(0, goo);
        }
        catch (Exception ex)
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new Guid("aa3e8488-943e-426f-b205-e8db5f684998");
}

public sealed class MotusUr10eRobotiqComponent : RobotSourceComponentBase
{
    public MotusUr10eRobotiqComponent()
        : base("Motus UR10e Robotiq", "UR10e", "Bundled UR10e arm with Robotiq 2F-85 gripper", "robot") { }

    protected override void RegisterInputParams(GH_InputParamManager p) { }

    protected override void RegisterOutputParams(GH_OutputParamManager p) =>
        p.AddGenericParameter("Robot", "Rb", "Robot model with URDF kinematics chain", GH_ParamAccess.item);

    protected override void SolveInstance(IGH_DataAccess da)
    {
        try
        {
            var path = BundledToolLoader.ResolveBundledPath(BundledToolLoader.Ur10eRobotiqUrdf);
            var goo = UrdfRobotLoad.Load(path, "base_link", "tool0");
            goo.EnsureBundledTool();
            if (goo.Tool?.Geometry is null)
            {
                var reason = BundledToolLoader.TryLoadRobotiqFailureReason(path)
                    ?? "Robotiq gripper mesh could not be loaded.";
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, reason);
            }
            ApplyPreview(goo, path);
            da.SetData(0, goo);
        }
        catch (Exception ex)
        {
            ClearPreview();
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, ex.Message);
        }
    }

    public override Guid ComponentGuid => new Guid("84b06a7d-8a3d-46ec-968f-25e74c249ad1");
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

        var plane = KinematicsPreview.TcpPlane(ctx.EffectiveModel, stateGoo.Value, ctx.Chain, ctx.Base, ctx.Tool);
        da.SetData(0, plane);
    }

    public override Guid ComponentGuid => new Guid("f1a2b3c4-d5e6-4789-a123-4567890abcde");
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
        p.AddTextParameter("ToolStates", "Ts", "Tool state JSON per waypoint", GH_ParamAccess.list);
        p[p.ParamCount - 1].Optional = true;
    }
    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!GhExtract.TryTrajectoryGoo(da, 0, out var trajGoo)) return;
        var t = trajGoo.Value!;
        var ctx = trajGoo.Context();
        if (t.Points.Count == 0) { AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Trajectory has no points."); return; }

        var planes = t.Points.Select(pt =>
            KinematicsPreview.TcpPlane(ctx.EffectiveModel, pt.JointState, ctx.Chain, ctx.Base, ctx.Tool)).ToList();
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
        if (t.Points.Any(p => p.ToolState is not null))
        {
            var toolStates = t.Points.Select(p =>
                p.ToolState is null
                    ? string.Empty
                    : System.Text.Json.JsonSerializer.Serialize(p.ToolState.Values)).ToList();
            da.SetDataList(3, toolStates);
        }
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
        if (!GhExtract.TryTrajectoryGoo(da, 0, out var trajGoo)) return;
        var t = trajGoo.Value!;
        var ctx = trajGoo.Context();
        var retime = true;
        var validate = false;
        da.GetData(1, ref retime);
        da.GetData(2, ref validate);

        var result = TrajectoryExport.Export(t, new TrajectoryExportOptions
        {
            Retime = retime,
            Validate = validate,
            SessionToolFrame = ctx.Tool,
            ToolCapabilities = trajGoo.ToolCapabilitiesSnapshot,
            Diagnostics = trajGoo.DiagnosticsSnapshot,
            Provenance = trajGoo.ProvenanceSnapshot
        });
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
